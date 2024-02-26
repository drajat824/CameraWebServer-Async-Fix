#include <ArduinoWebsockets.h>

#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "camera_index.h"
#include "Arduino.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"
#include <ESPmDNS.h>

const char *ssid = "DLB94859";
const char *password = "12345678";

IPAddress local_IP(192, 168, 43, 11);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 43, 1);
IPAddress primaryDNS(8, 8, 8, 8);

String hostname = "dlbcam";

#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

using namespace websockets;
WebsocketsServer socket_server;

camera_fb_t *fb = NULL;

long current_millis;
long last_detected_millis = 0;

unsigned long door_opened_millis = 0;
long interval = 5000;  // open lock for ... milliseconds
bool face_recognised = false;

void app_facenet_main();
void app_httpserver_init();

typedef struct
{
  uint8_t *image;
  box_array_t *net_boxes;
  dl_matrix3d_t *face_id;
} http_img_process_result;


static inline mtmn_config_t app_mtmn_config() {
  mtmn_config_t mtmn_config = { 0 };
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();

face_id_name_list st_face_list;
static dl_matrix3du_t *aligned_face = NULL;

httpd_handle_t camera_httpd = NULL;

typedef enum {
  START_STREAM,
  START_DETECT,
  SHOW_FACES,
  START_RECOGNITION,
  START_ENROLL,
  ENROLL_COMPLETE,
  DELETE_ALL,
} en_fsm_state;
en_fsm_state g_state;

typedef struct
{
  char enroll_name[ENROLL_NAME_LEN];
} httpd_resp_value;

httpd_resp_value st_name;

void setup_wifi() {

  Serial.println("Reconnect..");

  WiFi.config(local_IP, gateway, subnet, primaryDNS);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  if (!MDNS.begin("dlbcam")) {
    Serial.println("Error starting mDNS");
    return;
  }

  Serial.println("");
  Serial.println("WiFi connected");

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  setup_wifi();

  app_httpserver_init();
  app_facenet_main();
  socket_server.listen(82);
}

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}

httpd_uri_t index_uri = {
  .uri = "/",
  .method = HTTP_GET,
  .handler = index_handler,
  .user_ctx = NULL
};

void app_httpserver_init() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  if (httpd_start(&camera_httpd, &config) == ESP_OK)
    Serial.println("httpd_start");
  {
    httpd_register_uri_handler(camera_httpd, &index_uri);
  }
}

void app_facenet_main() {
  face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
  read_face_id_from_flash_with_name(&st_face_list);
}

static inline int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id) {
  ESP_LOGD(TAG, "START ENROLLING");
  int left_sample_face = enroll_face_id_to_flash_with_name(face_list, new_id, st_name.enroll_name);
  ESP_LOGD(TAG, "Face ID %s Enrollment: Sample %d",
           st_name.enroll_name,
           ENROLL_CONFIRM_TIMES - left_sample_face);
  return left_sample_face;
}

static esp_err_t send_face_list(WebsocketsClient &client) {
  client.send("delete_faces");  // tell browser to delete all faces
  face_id_node *head = st_face_list.head;
  char add_face[64];
  for (int i = 0; i < st_face_list.count; i++)  // loop current faces
  {
    sprintf(add_face, "listface:%s", head->id_name);
    client.send(add_face);  //send face to browser
    head = head->next;
  }
}

static esp_err_t delete_all_faces(WebsocketsClient &client) {
  delete_face_all_in_flash_with_name(&st_face_list);
  client.send("delete_faces");
}

void handle_message(WebsocketsClient &client, WebsocketsMessage msg) {
  if (msg.data() == "stream") {
    g_state = START_STREAM;
    client.send("STREAMING");
  }
  if (msg.data() == "detect") {
    g_state = START_DETECT;
    client.send("MULAI MENDETEKSI...");
  }
  if (msg.data().substring(0, 8) == "capture:") {
    g_state = START_ENROLL;
    char person[FACE_ID_SAVE_NUMBER * ENROLL_NAME_LEN] = {
      0,
    };
    msg.data().substring(8).toCharArray(person, sizeof(person));
    memcpy(st_name.enroll_name, person, strlen(person) + 1);
    client.send("MULAI MENGAMBIL GAMBAR...");
  }
  if (msg.data() == "recognise") {
    g_state = START_RECOGNITION;
    client.send("MULAI VERIFIKASI WAJAH...");
  }
  if (msg.data().substring(0, 7) == "remove:") {
    char person[ENROLL_NAME_LEN * FACE_ID_SAVE_NUMBER];
    msg.data().substring(7).toCharArray(person, sizeof(person));
    delete_face_id_in_flash_with_name(&st_face_list, person);
    send_face_list(client);  // reset faces in the browser
  }
  if (msg.data() == "delete_all") {
    delete_all_faces(client);
  }
}

// // Fungsi tambahan untuk menunggu koneksi client dengan batas waktu
// WebsocketsClient waitForClientConnection(unsigned long timeout) {
//   unsigned long startTime = millis();
//   WebsocketsClient client;

//   while (!client.available() && (millis() - startTime < timeout)) {
//     client = socket_server.accept();
//     delay(100);  // Jeda kecil untuk mengurangi beban CPU
//   }

//   return client;
// }

void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("NYAMBUNG LAGI OY");
    setup_wifi();
  }

  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);
  http_img_process_result out_res = { 0 };
  out_res.image = image_matrix->item;

  if (socket_server.poll()) {

    auto client = socket_server.accept();

    if (client.available()) {
      client.onMessage(handle_message);
      send_face_list(client);
      client.send("STREAMING");
    }

    while (client.available()) {
      client.poll();
      delay(500);

      fb = esp_camera_fb_get();

      if (g_state == START_DETECT || g_state == START_ENROLL || g_state == START_RECOGNITION) {
        out_res.net_boxes = NULL;
        out_res.face_id = NULL;

        fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image);

        out_res.net_boxes = face_detect(image_matrix, &mtmn_config);

        if (out_res.net_boxes) {
          if (align_face(out_res.net_boxes, image_matrix, aligned_face) == ESP_OK) {

            out_res.face_id = get_face_id(aligned_face);
            last_detected_millis = millis();
            if (g_state == START_DETECT) {
              client.send("WAJAH TERDETEKSI");
            }

            if (g_state == START_ENROLL) {
              int left_sample_face = do_enrollment(&st_face_list, out_res.face_id);
              char enrolling_message[64];
              sprintf(enrolling_message, "MENGAMBIL SAMPEL KE- %d UNTUK : %s", ENROLL_CONFIRM_TIMES - left_sample_face, st_name.enroll_name);
              client.send(enrolling_message);
              if (left_sample_face == 0) {
                ESP_LOGI(TAG, "Enrolled Face ID: %s", st_face_list.tail->id_name);
                g_state = START_STREAM;
                char captured_message[64];
                sprintf(captured_message, "WAJAH DIDAFTARKAN UNTUK : %s", st_face_list.tail->id_name);
                client.send(captured_message);
                send_face_list(client);
              }
            }

            if (g_state == START_RECOGNITION && (st_face_list.count > 0)) {
              face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id);
              if (f) {
                char recognised_message[64];
                sprintf(recognised_message, "WAJAH SESUAI DENGAN : %s", f->id_name);
                Serial.println(200);
                client.send(recognised_message);
              } else {
                Serial.println(403);
                client.send("WAJAH TIDAK SESUAI");
              }
            }
            dl_matrix3d_free(out_res.face_id);
          }

        } else {
          if (g_state != START_DETECT) {
            Serial.println(404);
            client.send("WAJAH TIDAK TERDETEKSI");
          }
        }

        if (g_state == START_DETECT && millis() - last_detected_millis > 500) {  // Detecting but no face detected
          client.send("MENDETEKSI...");
        }
      }

      client.sendBinary((const char *)fb->buf, fb->len);

      esp_camera_fb_return(fb);
      fb = NULL;
    }

  } else {

    while (true) {
      delay(500);

      fb = esp_camera_fb_get();

      out_res.net_boxes = NULL;
      out_res.face_id = NULL;

      fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image);

      out_res.net_boxes = face_detect(image_matrix, &mtmn_config);

      if (out_res.net_boxes) {
        if (align_face(out_res.net_boxes, image_matrix, aligned_face) == ESP_OK) {

          out_res.face_id = get_face_id(aligned_face);
          last_detected_millis = millis();

          if (st_face_list.count > 0) {
            face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id);
            if (f) {
              Serial.println(200);
            } else {
              Serial.println(403);
            }
          }
          dl_matrix3d_free(out_res.face_id);
        }

      } else {
        if (g_state != START_DETECT) {
          Serial.println(404);
        }
      }

      esp_camera_fb_return(fb);
      fb = NULL;

      if (socket_server.poll()) {
        break;
      }
    }
    
  }
}