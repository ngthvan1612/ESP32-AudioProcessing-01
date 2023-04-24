// #define TESTING_GRAP

// #ifdef TESTING_GRAPH
// #include <WiFi.h>
// #include <WebServer.h>
// #include <WebSocketsServer.h>
// #include <Ticker.h>
// #include <arduinoFFT.h>
// #include <driver/i2s.h>

// const i2s_port_t I2S_PORT = I2S_NUM_0;
// const int BLOCK_SIZE = 512;

// const double signalFrequency = 1000;
// const double samplingFrequency = 10000;
// const uint8_t amplitude = 150;

// double vReal[BLOCK_SIZE];
// double vImag[BLOCK_SIZE];
// int32_t samples[BLOCK_SIZE];

// String labels[] = {"125", "250", "500", "1K", "2K", "4K", "8K", "16K"};

// arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

// // Connecting to the Internet
// const char * ssid = "Arduino-71";
// const char * password = "12345678abc";

// int bands[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// // Running a web server
// WebServer server(80);

// // Adding a websocket to the server
// WebSocketsServer webSocket = WebSocketsServer(81);

// // Serving a web page (from flash memory)
// // formatted as a string literal!
// char webpage[] PROGMEM = R"=====(
// <html>
// <!-- Adding a data chart using Chart.js -->
// <head>
//   <script src='https://cdnjs.cloudflare.com/ajax/libs/Chart.js/2.5.0/Chart.min.js'></script>
// </head>
// <body onload="javascript:init()">
// <h2>Browser Based ESP32-EYE Spectrum Analyzer</h2>
// <div>
//   <canvas id="chart" width="600" height="400"></canvas>
// </div>
// <!-- Adding a websocket to the client (webpage) -->
// <script>
//   var webSocket, dataPlot;
//   var maxDataPoints = 20;
//   const maxValue = 200000000;
//   const maxLow = maxValue * 0.5;
//   const maxMedium = maxValue * 0.2;
//   const maxHigh = maxValue * 0.3;

//   function init() {
//     webSocket = new WebSocket('ws://' + window.location.hostname + ':81/');
//     dataPlot = new Chart(document.getElementById("chart"), {
//       type: 'bar',
//       data: {
//         labels: [],
//         datasets: [{
//           data: [],
//           label: "Low",
//           backgroundColor: "#D6E9C6"
//         },
//         {
//           data: [],
//           label: "Moderate",
//           backgroundColor: "#FAEBCC"
//         },
//         {
//           data: [],
//           label: "High",
//           backgroundColor: "#EBCCD1"
//         },
//         ]
//       }, 
//       options: {
//           responsive: false,
//           animation: false,
//           scales: {
//               xAxes: [{ stacked: true }],
//               yAxes: [{
//                   display: true,
//                   stacked: true,
//                   ticks: {
//                     beginAtZero: true,
//                     steps: 1000,
//                     stepValue: 500,
//                     max: maxValue
//                   }
//               }]
//            }
//        }
//     });
//     webSocket.onmessage = function(event) {
//       var data = JSON.parse(event.data);
//       dataPlot.data.labels = [];
//       dataPlot.data.datasets[0].data = [];
//       dataPlot.data.datasets[1].data = [];
//       dataPlot.data.datasets[2].data = [];
      
//       data.forEach(function(element) {
//         dataPlot.data.labels.push(element.bin);
//         var lowValue = Math.min(maxLow, element.value);
//         dataPlot.data.datasets[0].data.push(lowValue);
        
//         var mediumValue = Math.min(Math.max(0, element.value - lowValue), maxMedium);
//         dataPlot.data.datasets[1].data.push(mediumValue);
        
//         var highValue = Math.max(0, element.value - lowValue - mediumValue);
//         dataPlot.data.datasets[2].data.push(highValue);

//       });
//       dataPlot.update();
//     }
//   }

// </script>
// </body>
// </html>
// )=====";


// void getData() {

//   String json = "[";
//   for (int i = 0; i < 8; i++) {
//     if (i > 0) {
//       json +=", ";
//     }
//     json += "{\"bin\":";
//     json += "\"" + labels[i] + "\"";
//     json += ", \"value\":";
//     json += String(bands[i]);
//     json += "}"; 
//   }
//   json += "]";
//   webSocket.broadcastTXT(json.c_str(), json.length());
// }

// void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length){
//   // Do something with the data from the client
//   if(type == WStype_TEXT){

//   }
// }


// void setupMic() {
//   Serial.println("Configuring I2S...");
//   esp_err_t err;

//   // The I2S config as per the example
//   const i2s_config_t i2s_config = {
//       .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
//       .sample_rate = samplingFrequency,                        
//       .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // could only get it to work with 32bits
//       .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, // although the SEL config should be left, it seems to transmit on right
//       .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
//       .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
//       .dma_buf_count = 4,                           // number of buffers
//       .dma_buf_len = BLOCK_SIZE                     // samples per buffer
//   };

//   // The pin config as per the setup
//         // The pin config as per the setup
//     const i2s_pin_config_t pin_config = {
//         .bck_io_num = 2,   // SCK
//         .ws_io_num = 15,    // WS
//         .data_out_num = -1, // Data out
//         .data_in_num = 13   // SD
//     };

//   // Configuring the I2S driver and pins.
//   // This function must be called before any I2S driver read/write operations.
//   err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
//   if (err != ESP_OK) {
//     Serial.printf("Failed installing driver: %d\n", err);
//     while (true);
//   }
//   err = i2s_set_pin(I2S_PORT, &pin_config);
//   if (err != ESP_OK) {
//     Serial.printf("Failed setting pin: %d\n", err);
//     while (true);
//   }
//   Serial.println("I2S driver installed.");
// }



// void setup() {
//   // put your setup code here, to run once:
//   WiFi.begin(ssid, password);
//   Serial.begin(115200);
//   while(WiFi.status()!=WL_CONNECTED) {
//     Serial.print(".");
//     delay(500);
//   }
//   Serial.println("");
//   Serial.print("IP Address: ");
//   Serial.println(WiFi.localIP());

  
//   delay(1000);
//   Serial.println("Setting up mic");
//   setupMic();
//   Serial.println("Mic setup completed");

//   delay(1000);

//   server.on("/",[](){
//     server.send_P(200, "text/html", webpage);
//   });
//   server.begin();
//   webSocket.begin();
//   webSocket.onEvent(webSocketEvent);
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   webSocket.loop();
//   server.handleClient();

//   // Read multiple samples at once and calculate the sound pressure

//   int num_bytes_read;
//   i2s_read(I2S_PORT, 
//                                       (char *)samples, 
//                                       BLOCK_SIZE, (size_t*)&num_bytes_read,     // the doc says bytes, but its elements.
//                                       portMAX_DELAY); // no timeout

//   for (uint16_t i = 0; i < BLOCK_SIZE; i++) {
//     vReal[i] = samples[i] << 8;
//     vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
//   }

//   FFT.Windowing(vReal, BLOCK_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
//   FFT.Compute(vReal, vImag, BLOCK_SIZE, FFT_FORWARD);
//   FFT.ComplexToMagnitude(vReal, vImag, BLOCK_SIZE);
//   for (int i = 0; i < 8; i++) {
//     bands[i] = 0;
//   }
  
//   for (int i = 2; i < (BLOCK_SIZE/2); i++){ // Don't use sample 0 and only first SAMPLES/2 are usable. Each array eleement represents a frequency and its value the amplitude.
//     if (vReal[i] > 8000) { // Add a crude noise filter, 10 x amplitude or more
//       if (i<=2 )             bands[0] = max(bands[0], (int)(vReal[i]/amplitude)); // 125Hz
//       if (i >3   && i<=5 )   bands[1] = max(bands[1], (int)(vReal[i]/amplitude)); // 250Hz
//       if (i >5   && i<=7 )   bands[2] = max(bands[2], (int)(vReal[i]/amplitude)); // 500Hz
//       if (i >7   && i<=15 )  bands[3] = max(bands[3], (int)(vReal[i]/amplitude)); // 1000Hz
//       if (i >15  && i<=30 )  bands[4] = max(bands[4], (int)(vReal[i]/amplitude)); // 2000Hz
//       if (i >30  && i<=53 )  bands[5] = max(bands[5], (int)(vReal[i]/amplitude)); // 4000Hz
//       if (i >53  && i<=200 ) bands[6] = max(bands[6], (int)(vReal[i]/amplitude)); // 8000Hz
//       if (i >200           ) bands[7] = max(bands[7], (int)(vReal[i]/amplitude)); // 16000Hz
//     }

//     //for (byte band = 0; band <= 6; band++) display.drawHorizontalLine(18*band,64-peak[band],14);
//   }
//   getData();
//   /*for (int i = 0; i < 8; i++) {
//     Serial.print(String(bands[i]));
//     Serial.print(",");
//   }
//   Serial.println();*/
// }

// #elif TEST_01

#include <driver/adc.h>
#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include "Wav.h"
#include <WebServer.h>

#define I2S_WS 15
#define I2S_SD 13
#define I2S_SCK 2
#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE (16000)
#define I2S_SAMPLE_BITS (16)
#define I2S_READ_LEN (16 * 1024)
#define RECORD_TIME (10)
#define I2S_CHANNEL_NUM (1)
#define FLASH_RECORD_SIZE (I2S_CHANNEL_NUM * I2S_SAMPLE_BITS * I2S_SAMPLE_RATE / 8 * RECORD_TIME)

void i2s_init() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BITS),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 64,
    .dma_buf_len = 1024,
    .use_apll = 1
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}

void i2s_adc_data_scale(uint8_t* d_buff, uint8_t* s_buff, int32_t len) {
  uint32_t j = 0;
  uint32_t dac_value = 0;
  for (int i = 0; i < len; i += 2) {
    dac_value = (((uint16_t)(s_buff[i + 1] & 0xF) << 8) | (s_buff[i + 0]));
    d_buff[j++] = 0;
    d_buff[j++] = dac_value * 256 / 2048;
  }
}

void i2s_adc_dac(void* arg) {
  byte wavHeader[44];
  CreateWavHeader(wavHeader, FLASH_RECORD_SIZE);
  if (SPIFFS.exists("/record.wav"))
    SPIFFS.remove("/record.wav");
  File file = SPIFFS.open("/record.wav", "w");
  file.write(wavHeader, 44);

  int i2s_read_len = I2S_READ_LEN;
  int flash_wr_size = 0;
  size_t bytes_read;
  char* i2s_read_buff = (char*)calloc(i2s_read_len, sizeof(char));
  uint8_t* flash_write_buff = (uint8_t*)calloc(i2s_read_len, sizeof(char));

  Serial.println("Recording started...");
  while (flash_wr_size < FLASH_RECORD_SIZE) {
    i2s_read(I2S_PORT, (void*)i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
    i2s_adc_data_scale(flash_write_buff, (uint8_t*)i2s_read_buff, i2s_read_len);
    file.write(flash_write_buff, i2s_read_len);
    flash_wr_size += bytes_read;
    Serial.printf("Sound recording %d %d %d\n", bytes_read, i2s_read_len, 100 * flash_wr_size / FLASH_RECORD_SIZE);
  }

  file.close();

  free(i2s_read_buff);
  free(flash_write_buff);
  vTaskDelete(NULL);
}

WebServer server(80);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  i2s_init();

  Serial.print("Init SPIFFS...");
  if (!SPIFFS.begin(true)) {
    Serial.println("FAIL");
    delay(100);
  }
  while (!SPIFFS.begin());
  Serial.println("OK");

  Serial.print("Connecting wifi...");
  WiFi.begin("Arduino-71", "12345678abc");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(200);
  }
  Serial.println("\nIP = " + WiFi.localIP().toString());

  server.on("/download", []() {{
    File file = SPIFFS.open("/record.wav", "r");
    server.sendHeader("Content-Type", "text/text");
    server.sendHeader("Content-Disposition", "attachment; filename=ahihi.wav");
    server.sendHeader("Connection", "close");
    server.streamFile(file, "application/octet-stream");
    file.close();
  }});

  xTaskCreate(i2s_adc_dac, "vidu_i2s_adc_dac", 1024 * 2, NULL, 5, NULL);

  server.begin();
}

void loop() {
  server.handleClient();
}

// // #else

// #include <driver/i2s.h>
// #include <WiFiUdp.h>
// #include "WiFi.h"
// #include "AsyncUDP.h"

// #define PIN_CLK  0
// #define PIN_DATA 34
// #define READ_LEN (2 * 512)
// #define ESP_NOW_MAX_DATA_LEN READ_LEN
// #define GAIN_FACTOR 3
// uint8_t BUFFER[READ_LEN / 2 * 4] = {0};

// size_t bytesRead = 0;
// uint8_t buffer32[ESP_NOW_MAX_DATA_LEN * 4] = {0};

// uint16_t sound_wav[1024];
// int bytesPacked = 0;
// int16_t *adcBuffer = NULL;

// AsyncUDP udp;
// //WiFiUDP udp;

// #define I2S_WS 15
// #define I2S_SD 13
// #define I2S_SCK 2

// const char * ssid = "Arduino-71";
// const char * password = "12345678abc";

// void setupMic() {
//   const i2s_config_t i2s_config = {
//       .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
//       .sample_rate = 44100,                        
//       .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // could only get it to work with 32bits
//       .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // although the SEL config should be left, it seems to transmit on right
//       .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
//       .intr_alloc_flags = 0,     // Interrupt level 1
//       .dma_buf_count = 64,                           // number of buffers
//       .dma_buf_len = 1024                     // samples per buffer
//   };

//   i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);

//   const i2s_pin_config_t pin_config = {
//     .bck_io_num = I2S_SCK,
//     .ws_io_num = I2S_WS,
//     .data_out_num = -1,
//     .data_in_num = I2S_SD
//   };

//   i2s_set_pin(I2S_NUM_0, &pin_config);
// }


// void i2sInit()
// {
//   setupMic();
// }

// void showSignal();

// void mic_record_task (void* arg)
// {   
//   size_t bytesread;
//   while(1){
//     //i2s_read(I2S_NUM_0, &buffer32, sizeof(buffer32), &bytesRead, portMAX_DELAY);
//     //int samplesRead = bytesRead / 4;
//     i2s_read(I2S_NUM_0, (char*) BUFFER, READ_LEN, &bytesread, portMAX_DELAY);
//     showSignal();
//     //vTaskDelay(10 / portTICK_RATE_MS);
//   }
// }

// void i2s_adc_data_scale(uint8_t* d_buff, uint8_t* s_buff, int32_t len) {
//   uint32_t j = 0;
//   uint32_t dac_value = 0;
//   for (int i = 0; i < len; i += 2) {
//     dac_value = (((uint16_t)(s_buff[i + 1] & 0xF) << 8) | (s_buff[i + 0]));
//     d_buff[j++] = 0;
//     d_buff[j++] = dac_value;
//   }
// }

// void setup() {
//   Serial.begin(115200);

//   WiFi.mode(WIFI_STA);
//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) {
//     Serial.print(".");
//     delay(500);
//   }

//   udp.connect(IPAddress(192, 168, 137, 1), (uint16_t)3006);
 
//   i2sInit();
//   xTaskCreate(mic_record_task, "mic_record_task", 20480, NULL, 1, NULL);
// }


// void showSignal(){
//   i2s_adc_data_scale((uint8_t*)sound_wav, (uint8_t*)BUFFER, READ_LEN);

//   Serial.println(String("Sent at ") + millis());

//   udp.write((uint8_t*)sound_wav, READ_LEN);
// }

// void loop() {
//   //printf("loop cycling\n");
//   //vTaskDelay(10000 / portTICK_RATE_MS); // otherwise the main task wastes half of the cpu cycles
// }

// //#endif
