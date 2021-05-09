#include <stdio.h>
#include <string.h>

#include "mbed.h"
#include "uLCD_4DGL.h"
#include "mbed_rpc.h"

#include "stm32l475e_iot01_accelero.h"
#include "accelerometer_handler.h"

#include "config.h"
#include "magic_wand_model_data.h"
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

using namespace std::chrono;
 
DigitalOut myled1(LED1);
DigitalOut myled2(LED2);
DigitalOut myled3(LED3);//-------------------------
BufferedSerial pc(USBTX, USBRX);

uLCD_4DGL uLCD(D1, D0, D2);
InterruptIn sw3(USER_BUTTON);
Thread t0;
Thread t1;
Thread t2;

void gesture(Arguments *in, Reply *out);
void tilt(Arguments *in, Reply *out);
RPCFunction gesture_UI(&gesture, "gesture");
RPCFunction tilt_angle_detection(&tilt, "tilt");
char bufff[256];
int angle_index=0;
int function_index=0;

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
    char buf5[256];
    memset(buf5, 0, 256);
    strcpy(buf5,bufff);
    printf(buf5);
    char outbuf[256];
    //Call the static call method on the RPC class
    RPC::call(buf5, outbuf);
    printf("%s\r\n", outbuf);
}

void choose_angle()
{
    myled3=1;
    printf("num= %d",angle_index);
}

void dct()
{
    function_index+=1;
    myled3=!myled3;
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
        return -1;
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
        return -1;
    }

    int input_length = model_input->bytes / sizeof(float);

    TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
    if (setup_status != kTfLiteOk) {
        error_reporter->Report("Set up failed\n");
        return -1;
    }

    //error_reporter->Report("Set up successful...\n");

    while (true) {

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
        if (gesture_index < label_num) {
            if(function_index==1)
            {
                strcpy(bufff,config.output_message[gesture_index]);
                t1.start(choose_function);
                break;
            }
            else if(function_index==2)
            {   
                if(angle_index<4)
                    angle_index+=1;
                else
                    angle_index=1;
                strcpy(bufff,config.output_message[gesture_index]);
                if(angle_index==1)
                {
                    uLCD.cls();
                    uLCD.printf("\ngesture UI:\n");
                    uLCD.printf("\nselect threshold angle\n");
                    uLCD.printf("\n30 <--\n");
                    uLCD.printf("\n45\n");
                    uLCD.printf("\n60\n");
                    uLCD.printf("\n90\n");
                    sw3.rise(choose_angle);
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
                    sw3.rise(choose_angle);
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
                    sw3.rise(choose_angle);
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
                    sw3.rise(choose_angle);
                }
            }
                //error_reporter->Report(config.output_message[gesture_index]);
        }
    }
}


int main(int argc, char* argv[])
{
    uLCD.printf("\nSelection:\n"); //Default Green on black text
    uLCD.printf("\ngesture UI\n");
    uLCD.printf("\ntilt angle detection\n");
    
    myled3=0;
    t0.start(dct);
/*
    strcpy(bufff,"/gesture/run\n\r");
    t1.start(choose_function);
*/    
    
}
void gesture (Arguments *in, Reply *out)   {
    bool success = true;
    
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

    t2.start(dct);
/*    
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
        return -1;
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
        return -1;
    }

    int input_length = model_input->bytes / sizeof(float);

    TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
    if (setup_status != kTfLiteOk) {
        error_reporter->Report("Set up failed\n");
        return -1;
    }

    //error_reporter->Report("Set up successful...\n");

    while (true) {

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
        if (gesture_index < label_num) { 
            //myled3=1;
            if(angle_index<4)
                angle_index+=1;
            else
                angle_index=1;
            strcpy(bufff,config.output_message[gesture_index]);
            if(angle_index==1)
            {
                uLCD.cls();
                uLCD.printf("\ngesture UI:\n");
                uLCD.printf("\nselect threshold angle\n");
                uLCD.printf("\n30 <--\n");
                uLCD.printf("\n45\n");
                uLCD.printf("\n60\n");
                uLCD.printf("\n90\n");
                sw3.rise(choose_angle);
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
                sw3.rise(choose_angle);
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
                sw3.rise(choose_angle);
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
                sw3.rise(choose_angle);
            }
            //error_reporter->Report(config.output_message[gesture_index]);
        }
    }
    */
}
void tilt (Arguments *in, Reply *out)   {
    bool success = true;

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
}
