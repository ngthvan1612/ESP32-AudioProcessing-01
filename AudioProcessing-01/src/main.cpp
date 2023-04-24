#include <driver/adc.h>
#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include "Wav.h"
#include "AsyncUDP.h"
#include <WebServer.h>

class FixedAsyncUDPMessage : public AsyncUDPMessage {
public:
  FixedAsyncUDPMessage(size_t size=CONFIG_TCP_MSS) {
    this->_index = 0;
    this->_size = size;
    this->_buffer = (uint8_t *)malloc(size);
  }
};

#define I2S_WS 15
#define I2S_SD 13
#define I2S_SCK 2
#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE (16000)
#define I2S_SAMPLE_BITS (16)
#define I2S_READ_LEN (1024)
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

void record_and_save_wav() {
  WebServer server(80);

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

  server.on("/download", [&]() {{
    File file = SPIFFS.open("/record.wav", "r");
    server.sendHeader("Content-Type", "text/text");
    server.sendHeader("Content-Disposition", "attachment; filename=ahihi.wav");
    server.sendHeader("Connection", "close");
    server.streamFile(file, "application/octet-stream");
    file.close();
  }});

  xTaskCreate(i2s_adc_dac, "vidu_i2s_adc_dac", 1024 * 2, NULL, 5, NULL);

  server.begin();

  //LOOP: server.handleClient();
}

AsyncUDP udp;

int counter = 0;

void udp_client_test() {
  uint8_t buffer[16];

  Serial.begin(115200);
  while (!Serial);

  WiFi.mode(WIFI_STA);
  WiFi.begin("Arduino-71", "12345678abc");
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("WiFi Failed");
      while(1) {
          delay(1000);
      }
  }

  if (udp.connect(IPAddress(192, 168, 225, 91), 1234)) {
    while (true) {
      Serial.println("Connect to server 1234 ok");
      AsyncUDPMessage message;
      buffer[0] = (uint8_t)(++counter);
      buffer[15] = (uint8_t)(++counter);
      message.write(buffer, 16);
      udp.send(message);
      delay(1000);
    }
  }
}

const int MAXIMUM_MESSAGE_LENGTH = 0;

void i2s_adc_dac_udp(void* arg) {
  int i2s_read_len = I2S_READ_LEN;
  int flash_wr_size = 0;
  size_t bytes_read;
  char* i2s_read_buff = (char*)calloc(i2s_read_len, sizeof(char));
  uint8_t* flash_write_buff = (uint8_t*)calloc(i2s_read_len, sizeof(char));

  Serial.println("Recording started...");
  while (1 < 2) {
    i2s_read(I2S_PORT, (void*)i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
    i2s_adc_data_scale(flash_write_buff, (uint8_t*)i2s_read_buff, i2s_read_len);

    //for (int i = 0; i < I2S_READ_LEN; i += 1024) {
    AsyncUDPMessage message;
    //message.write(i == 0);
    message.write(flash_write_buff, I2S_READ_LEN);
    udp.send(message);
    //}
    //message.~AsyncUDPMessage();

    flash_wr_size += bytes_read;
    Serial.printf("Sound recording %d bytes\n", flash_wr_size);
  }

  free(i2s_read_buff);
  free(flash_write_buff);
  vTaskDelete(NULL);
}

void setup() {
  i2s_init();

  Serial.begin(115200);
  while (!Serial);

  Serial.print("Connecting wifi...");
  WiFi.begin("Arduino-71", "12345678abc");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(200);
  }

  Serial.println("\nConnected wifi, ip = " + WiFi.localIP().toString());

  while (!udp.connect(IPAddress(192, 168, 225, 91), 1234)) {
    Serial.println("Connecting to udp server ...");
    delay(200);
  }

  Serial.println("Connect to udp server ok");

  xTaskCreate(i2s_adc_dac_udp, "vidu_i2s_adc_dac", 8128 * 2, NULL, 5, NULL);
}

void loop() {

}
