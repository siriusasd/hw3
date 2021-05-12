#include "mbed.h"
#include "uLCD_4DGL.h"
#include "mbed_rpc.h"

#include "stm32l475e_iot01_accelero.h"
#include "accelerometer_handler.h"

#include "tfconfig.h"
#include "magic_wand_model_data.h"
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"

using namespace std::chrono;

WiFiInterface *wifi = WiFiInterface::get_default_instance();
int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
NetworkInterface* net = wifi;
MQTTNetwork mqttNetwork(net);
MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);

volatile int arrivedcount = 0;
volatile bool closed = false;

const char* topic = "Mbed";
 
DigitalOut myled1(LED1);
DigitalOut myled2(LED2);
DigitalOut myled3(LED3);//-------------------------
BufferedSerial pc(USBTX, USBRX);

Thread mqtt_thread(osPriorityHigh);
EventQueue mqtt_queue;
EventQueue queue;

uLCD_4DGL uLCD(D1, D0, D2);
InterruptIn sw3(USER_BUTTON);

Thread t0;
Thread t1;
Thread t2;
Thread t3;
Thread t4;

void gesture(Arguments *in, Reply *out);
void gstop(Arguments *in, Reply *out);
void tilt(Arguments *in, Reply *out);
void getAcc(Arguments *in, Reply *out);
RPCFunction gesture_UI(&gesture, "gesture");
RPCFunction gestureUI_stop(&gstop, "gestureUI_stop");
RPCFunction tilt_angle_detection(&tilt, "tilt");
RPCFunction rpcAcc(&getAcc, "getAcc");
char bufff[256];
char buf5[256];
char outbuf[256];
int angle_index=0;
int function_index=0;
int angle=0;
int mode=0;
int indct=0;

// Create an area of memory to use for input, output, and intermediate arrays.
// The size of this will depend on the model you're using, and may need to be
// determined by experimentation.
constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

// Return the result of the last prediction
int PredictGesture(float* output) {
    // How many times the most recent gesture has been matched in a row
    static int continuous_count = 0;
    // The result of the last prediction
    static int last_predict = -1;

    // Find whichever output has a probability > 0.8 (they sum to 1)
    int this_predict = -1;
    for (int i = 0; i < label_num; i++) {
        if (output[i] > 0.8) this_predict = i;
    }

    // No gesture was detected above the threshold
    if (this_predict == -1) {
        continuous_count = 0;
        last_predict = label_num;
        return label_num;
    }

    if (last_predict == this_predict) {
        continuous_count += 1;
    } else {
        continuous_count = 0;
    }
    last_predict = this_predict;

    // If we haven't yet had enough consecutive matches for this gesture,
    // report a negative result
    if (continuous_count < config.consecutiveInferenceThresholds[this_predict]) {
        return label_num;
    }
    // Otherwise, we've seen a positive result, so clear all our variables
    // and report it
    continuous_count = 0;
    last_predict = -1;

    return this_predict;
}

void choose_function()
{
    memset(buf5, 0, 256);
    memset(outbuf, 0, 256);
    strcpy(buf5,bufff);
    printf(buf5);
    
    //Call the static call method on the RPC class
    RPC::call(buf5, outbuf);
    printf("%s\r\n", outbuf);
}

void messageArrived(MQTT::MessageData& md) {
    MQTT::Message &message = md.message;
    char msg[300];
    sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
    printf(msg);
    ThisThread::sleep_for(1000ms);
    char payload[300];
    sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
    printf(payload);
    ++arrivedcount;
}

void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client) {
    indct=1;
    uLCD.cls();
    if(mode==1)
    {
        uLCD.printf("\ngesture UI:\n");
        uLCD.printf("\nselect threshold angle\n");
        if(angle_index==1)
        {
            uLCD.printf("\n30 is selected\n");
            angle=30;
        }     
        else if(angle_index==2)
        {
            uLCD.printf("\n45 is selected\n");
            angle=45;
        }
        else if(angle_index==3)
        {
            uLCD.printf("\n60 is selected\n");
            angle=60;
        } 
        else if(angle_index==4)
        {
            uLCD.printf("\n90 is selected\n");
            angle=90;
        }    
        
        MQTT::Message message;
        char buff[100];
        sprintf(buff, "QoS0 Hello, selected threshold angle: %d", angle);
        message.qos = MQTT::QOS0;
        message.retained = false;
        message.dup = false;
        message.payload = (void*) buff;
        message.payloadlen = strlen(buff) + 1;
        int rc = client->publish(topic, message);
        printf("rc:  %d\r\n", rc);
        printf("Puslish message: %s\r\n", buff);
    }
/*    else if(mode==2)
    {
        
    }
*/
    function_index=0;//---------------------------------------

    ThisThread::sleep_for(1s);
    sprintf(buf5,"/gestureUI_stop/run\n\r");
    RPC::call(buf5, outbuf);
    printf("%s\r\n", outbuf);
}

void close_mqtt() {
    closed = true;
}

void dct()
{
    function_index+=1;
    // Whether we should clear the buffer next time we fetch data
    bool should_clear_buffer = false;
    bool got_data = false;

    // The gesture index of the prediction
    int gesture_index;

    // Set up logging.
    static tflite::MicroErrorReporter micro_error_reporter;
    tflite::ErrorReporter* error_reporter = &micro_error_reporter;

    // Map the model into a usable data structure. This doesn't involve any
    // copying or parsing, it's a very lightweight operation.
    const tflite::Model* model = tflite::GetModel(g_magic_wand_model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        error_reporter->Report(
            "Model provided is schema version %d not equal "
            "to supported version %d.",
            model->version(), TFLITE_SCHEMA_VERSION);
        return;
    }

    // Pull in only the operation implementations we need.
    // This relies on a complete list of all the ops needed by this graph.
    // An easier approach is to just use the AllOpsResolver, but this will
    // incur some penalty in code space for op implementations that are not
    // needed by this graph.
    static tflite::MicroOpResolver<6> micro_op_resolver;
    micro_op_resolver.AddBuiltin(
        tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
        tflite::ops::micro::Register_DEPTHWISE_CONV_2D());
    micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
                                tflite::ops::micro::Register_MAX_POOL_2D());
    micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
                                tflite::ops::micro::Register_CONV_2D());
    micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
                                tflite::ops::micro::Register_FULLY_CONNECTED());
    micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                                tflite::ops::micro::Register_SOFTMAX());
    micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                                tflite::ops::micro::Register_RESHAPE(), 1);

    // Build an interpreter to run the model with
    static tflite::MicroInterpreter static_interpreter(
        model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
    tflite::MicroInterpreter* interpreter = &static_interpreter;

    // Allocate memory from the tensor_arena for the model's tensors
    interpreter->AllocateTensors();

    // Obtain pointer to the model's input tensor
    TfLiteTensor* model_input = interpreter->input(0);
    if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
        (model_input->dims->data[1] != config.seq_length) ||
        (model_input->dims->data[2] != kChannelNumber) ||
        (model_input->type != kTfLiteFloat32)) {
        error_reporter->Report("Bad input tensor parameters in model");
        return;
    }

    int input_length = model_input->bytes / sizeof(float);

    TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
    if (setup_status != kTfLiteOk) {
        error_reporter->Report("Set up failed\n");
        return;
    }

    //error_reporter->Report("Set up successful...\n");

    while (true) {
        //myled3=1;
        // Attempt to read new data from the accelerometer
        got_data = ReadAccelerometer(error_reporter, model_input->data.f,
                                    input_length, should_clear_buffer);

        // If there was no new data,
        // don't try to clear the buffer again and wait until next time
        if (!got_data) {
        should_clear_buffer = false;
        continue;
        }

        // Run inference, and report any error
        TfLiteStatus invoke_status = interpreter->Invoke();
        if (invoke_status != kTfLiteOk) {
        error_reporter->Report("Invoke failed on index: %d\n", begin_index);
        continue;
        }

        // Analyze the results to obtain a prediction
        gesture_index = PredictGesture(interpreter->output(0)->data.f);
        
        
        // Clear the buffer next time we read data
        should_clear_buffer = gesture_index < label_num;

        memset(bufff, 0, 256);
        // Produce an output
        if (gesture_index < label_num) 
        {
            //myled3=0;
            if(function_index==1)
            {
                strcpy(bufff,config.output_message[gesture_index]);
                t0.start(choose_function);

                break;
            }
            else if(function_index==2)
            {   
                if(angle_index<4)
                    angle_index+=1;
                else
                    angle_index=1;
                if(angle_index==1)
                {
                    uLCD.cls();
                    uLCD.printf("\ngesture UI:\n");
                    uLCD.printf("\nselect threshold angle\n");
                    uLCD.printf("\n30 <--\n");
                    uLCD.printf("\n45\n");
                    uLCD.printf("\n60\n");
                    uLCD.printf("\n90\n");
                    sw3.rise(mqtt_queue.event(&publish_message, &client));
                    if(indct==1)
                    {
                        break;
                    }
                }
                else if(angle_index==2)
                {
                    uLCD.cls();
                    uLCD.printf("\ngesture UI:\n");
                    uLCD.printf("\nselect threshold angle\n");
                    uLCD.printf("\n30\n");
                    uLCD.printf("\n45 <--\n");
                    uLCD.printf("\n60\n");
                    uLCD.printf("\n90\n");
                    sw3.rise(mqtt_queue.event(&publish_message, &client));
                    if(indct==1)
                    {
                        break;
                    }
                }
                else if(angle_index==3)
                {
                    uLCD.cls();
                    uLCD.printf("\ngesture UI:\n");
                    uLCD.printf("\nselect threshold angle\n");
                    uLCD.printf("\n30\n");
                    uLCD.printf("\n45\n");
                    uLCD.printf("\n60 <--\n");
                    uLCD.printf("\n90\n");
                    sw3.rise(mqtt_queue.event(&publish_message, &client));
                    if(indct==1)
                    {
                        break;
                    }
                }
                else if(angle_index==4)
                {
                    uLCD.cls();
                    uLCD.printf("\ngesture UI:\n");
                    uLCD.printf("\nselect threshold angle\n");
                    uLCD.printf("\n30\n");
                    uLCD.printf("\n45\n");
                    uLCD.printf("\n60\n");
                    uLCD.printf("\n90 <--\n");
                    sw3.rise(mqtt_queue.event(&publish_message, &client));
                    if(indct==1)
                    {
                        break;
                    }
                }
            }
                //error_reporter->Report(config.output_message[gesture_index]);
        }
    }
    return;
}

void choose()
{
    uLCD.cls();
    uLCD.printf("\nSelection:\n"); //Default Green on black text
    uLCD.printf("\ngesture UI\n");
    uLCD.printf("\ntilt angle detection\n");
    dct();
}

void init()
{
    //myled3=1;
    uLCD.printf("\ninitializing...\n");
    ThisThread::sleep_for(1s);
    BSP_ACCELERO_Init();
    uLCD.printf("\nfinished\n");
    //myled3=0;
    ThisThread::sleep_for(1s);
    uLCD.printf("\nstart\n");
    
    char b5[256];
    myled3=1;
    memset(buf5, 0, 256);
    memset(outbuf, 0, 256);
    sprintf(buf5, "/getAcc/run\n\r");
    strcpy(b5, buf5);
    printf(b5);
    
    //Call the static call method on the RPC class
    RPC::call(b5, outbuf);
    printf("%s\r\n", outbuf);

    return;
}

void getAcc(Arguments *in, Reply *out) {
   int16_t pDataXYZ[3] = {0};
        char buffer[200];
   BSP_ACCELERO_AccGetXYZ(pDataXYZ);
   sprintf(buffer, "Accelerometer values: (%d, %d, %d)", pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);
   out->putData(buffer);
}

int main(int argc, char* argv[])
{
    if (!wifi) {
        printf("ERROR: No WiFiInterface found.\r\n");
        return -1;
    }

    printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
    
    if (ret != 0) {
            printf("\nConnection error: %d\r\n", ret);
            return -1;
    }

    //TODO: revise host to your IP
    const char* host = "192.168.43.168";
    printf("Connecting to TCP network...\r\n");

    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);

    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting

    int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
    if (rc != 0) {
            printf("Connection error.");
            return -1;
    }
    printf("Successfully connected!\r\n");

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = "Mbed";

    if ((rc = client.connect(data)) != 0){
            printf("Fail to connect MQTT\r\n");
    }
    if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }

    mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));
    t3.start(callback(&queue, &EventQueue::dispatch_forever));

    choose();
}

void gesture (Arguments *in, Reply *out)   {
    bool success = true;
    mode = 1;
    char strings[50];
    char buffer[200];
    sprintf(strings, "gestureUI mode\n");
    strcpy(buffer, strings);
    if (success) {
        out->putData(buffer);
    } else {
        out->putData("Failed to execute.");
    }

    myled1=1;
    uLCD.cls();
    uLCD.printf("\ngesture UI:\n");
    uLCD.printf("\nselect threshold angle\n");
    uLCD.printf("\n30\n");
    uLCD.printf("\n45\n");
    uLCD.printf("\n60\n");
    uLCD.printf("\n90\n");
    t1.start(dct);
    return;
}

void gstop(Arguments *in, Reply *out)
{
    bool success = true;
    mode = 0;
    char strings[50];
    char buffer[200];
    sprintf(strings, "gestureUI mode stopped\n\r");
    strcpy(buffer, strings);
    if (success) {
        out->putData(buffer);
    } else {
        out->putData("Failed to execute.");
    }

    myled1=0;
    t2.start(choose);
    return;
}

void tilt (Arguments *in, Reply *out)   {
    bool success = true;
    mode=2;
    char strings[50];
    char buffer[200];
    sprintf(strings, "tilt angle detection mode\n");
    strcpy(buffer, strings);
    if (success) {
        out->putData(buffer);
    } else {
        out->putData("Failed to execute LED control.");
    }

    myled2=1;
    uLCD.cls();
    uLCD.printf("\ntilt angle detection\n");
    sw3.fall(queue.event(&init));
    return;
}