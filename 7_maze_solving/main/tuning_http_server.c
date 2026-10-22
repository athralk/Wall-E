#include <stdio.h>
#include <stdlib.h>
#include "tuning_http_server.h"
#include "line_sensor.h"
#include "maze.h"

static const char *TAG = "tuning_http_server";
static char scratch[SCRATCH_BUFSIZE];
// Starting gains. The error is how far the line is off centre, in sensors (-2..+2), so
// Kp 5 = 5 duty of steering per sensor of offset
static pid_const_t pid_constants = {.kp = 5, .ki = 0, .kd = 15, .val_changed = true};
static httpd_handle_t server_handle = NULL;
static volatile int ws_fd = -1;     // socket of the live LSA page (-1 = none open)

#define LSA_PUSH_MS 40              // live LSA page: a reading every 40 ms (25 a second)

static void initialise_mdns(void)
{
    mdns_init();
    mdns_hostname_set(MDNS_HOST_NAME);
    mdns_instance_name_set(MDNS_INSTANCE);

    mdns_txt_item_t serviceTxtData[] = {
        {"board", "esp32"},
        {"path", "/"}
    };

    ESP_ERROR_CHECK(mdns_service_add("ESP32-WebServer", "_http", "_tcp", 80, serviceTxtData,
                                     sizeof(serviceTxtData) / sizeof(serviceTxtData[0])));
}

static esp_err_t init_fs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = WEB_MOUNT_POINT,
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = false
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
    return ESP_OK;
}

/* Set HTTP response content type according to file extension */
static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filepath)
{
    const char *type = "text/plain";
    if (CHECK_FILE_EXTENSION(filepath, ".html")) {
        type = "text/html";
    } else if (CHECK_FILE_EXTENSION(filepath, ".js")) {
        type = "application/javascript";
    } else if (CHECK_FILE_EXTENSION(filepath, ".css")) {
        type = "text/css";
    } else if (CHECK_FILE_EXTENSION(filepath, ".png")) {
        type = "image/png";
    } else if (CHECK_FILE_EXTENSION(filepath, ".ico")) {
        type = "image/x-icon";
    } else if (CHECK_FILE_EXTENSION(filepath, ".svg")) {
        type = "text/xml";
    }
    return httpd_resp_set_type(req, type);
}

/* Send HTTP response with the contents of the requested file */
static esp_err_t rest_common_get_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX] = WEB_MOUNT_POINT;

    if (strlen(req->uri) > 0 && req->uri[strlen(req->uri) - 1] == '/') 
    {
        strlcat(filepath, "/index.html", sizeof(filepath));
    }
    else 
    {
        strlcat(filepath, req->uri, sizeof(filepath));
    }

    int fd = open(filepath, O_RDONLY, 0);
    if (fd == -1) {
        ESP_LOGE(TAG, "Failed to open file : %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    set_content_type_from_file(req, filepath);

    char *chunk = scratch;
    memset(scratch, '\0', SCRATCH_BUFSIZE);
    ssize_t read_bytes;
    do {
        /* Read file in chunks into the scratch buffer */
        read_bytes = read(fd, chunk, SCRATCH_BUFSIZE);
        if (read_bytes == -1) {
            ESP_LOGE(TAG, "Failed to read file : %s", filepath);
        } else if (read_bytes > 0) {
            /* Send the buffer contents as HTTP response chunk */
            if (httpd_resp_send_chunk(req, chunk, read_bytes) != ESP_OK) {
                close(fd);
                ESP_LOGE(TAG, "File sending failed!");
                /* Abort sending file */
                httpd_resp_sendstr_chunk(req, NULL);
                /* Respond with 500 Internal Server Error */
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
                return ESP_FAIL;
            }
        }
    } while (read_bytes > 0);
    /* Close file after sending complete */
    close(fd);
    ESP_LOGI(TAG, "File sending complete");
    /* Respond with an empty chunk to signal HTTP response completion */
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* Simple handler for Tuning PID */
static esp_err_t tuning_pid_post_handler(httpd_req_t *req)
{
    int total_len = req->content_len;
    int cur_len = 0;
    char *buf = scratch;
    memset(scratch, '\0', SCRATCH_BUFSIZE);
    int received = 0;
    if (total_len >= SCRATCH_BUFSIZE) {
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "content too long");
        return ESP_FAIL;
    }
    while (cur_len < total_len) {
        received = httpd_req_recv(req, buf + cur_len, total_len);
        if (received <= 0) {
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to post control value");
            return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[total_len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (root == NULL)
    {   
        ESP_LOGE(TAG, "invalid json response");
        return ESP_FAIL;
    }
    
    if (!cJSON_HasObjectItem(root, "kp") || !cJSON_HasObjectItem(root, "ki") || !cJSON_HasObjectItem(root, "kd"))
    {
        ESP_LOGE(TAG, "invalid json response");
        return ESP_FAIL;
    }
    
    pid_constants.kp = (float)cJSON_GetObjectItem(root, "kp")->valuedouble;
    pid_constants.ki = (float)cJSON_GetObjectItem(root, "ki")->valuedouble;
    pid_constants.kd = (float)cJSON_GetObjectItem(root, "kd")->valuedouble;

    cJSON_Delete(root);
    httpd_resp_sendstr(req, "Post control value successfully");

    pid_constants.val_changed = true;
    return ESP_OK;
}

/*
 * GET /api/v1/lsa: the latest sensor bar reading as JSON, for frontend/lsa.html
 * {"r":[5 readings 0-1000],"on":[5 x 0/1],"pos":line position,"state":"FOLLOW"}
 */
static esp_err_t lsa_get_handler(httpd_req_t *req)
{
    lsa_state_t l = line_sensor_latest();
    char json[160];
    snprintf(json, sizeof(json),
             "{\"r\":[%d,%d,%d,%d,%d],\"on\":[%d,%d,%d,%d,%d],\"pos\":%.2f,\"state\":\"%s\"}",
             l.reading[0], l.reading[1], l.reading[2], l.reading[3], l.reading[4],
             l.on_line[0], l.on_line[1], l.on_line[2], l.on_line[3], l.on_line[4],
             l.position, maze_state_text());
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json);
}

/*
 * /ws: the live LSA page (frontend/lsa.html) opens a WebSocket here and the bot pushes
 * every reading to it (lsa_push_task), no request per update. The page sends nothing.
 */
static esp_err_t lsa_ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        ws_fd = httpd_req_to_sockfd(req);   // handshake done: this is the page to push to
        ESP_LOGI(TAG, "live LSA page connected");
        return ESP_OK;
    }
    // Read (and drop) anything the page sends, so the socket stays healthy
    uint8_t buf[32];
    httpd_ws_frame_t frame = {.type = HTTPD_WS_TYPE_TEXT, .payload = buf};
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
    if (err == ESP_OK && frame.len > 0 && frame.len < sizeof(buf))
        err = httpd_ws_recv_frame(req, &frame, frame.len);
    return err;
}

/*
 * Runs in the server's own task (httpd_queue_work), where sending is safe
 */
static void ws_send_work(void *arg)
{
    char *msg = arg;
    httpd_ws_frame_t frame = {.type = HTTPD_WS_TYPE_TEXT, .payload = (uint8_t *)msg, .len = strlen(msg)};
    int fd = ws_fd;
    if (fd >= 0 && httpd_ws_send_frame_async(server_handle, fd, &frame) != ESP_OK)
        ws_fd = -1;     // page closed
    free(msg);
}

/*
 * One line per reading: "r0 r1 r2 r3 r4 pattern position state", left (U5) to right (U1)
 */
static void lsa_push_task(void *arg)
{
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(LSA_PUSH_MS));
        int fd = ws_fd;
        if (fd < 0 || httpd_ws_get_fd_info(server_handle, fd) != HTTPD_WS_CLIENT_WEBSOCKET)
        {
            ws_fd = -1;
            continue;
        }
        lsa_state_t l = line_sensor_latest();
        char *msg = malloc(96);
        if (msg == NULL)
            continue;
        snprintf(msg, 96, "%d %d %d %d %d %d%d%d%d%d %.2f %s",
                 l.reading[0], l.reading[1], l.reading[2], l.reading[3], l.reading[4],
                 l.on_line[0], l.on_line[1], l.on_line[2], l.on_line[3], l.on_line[4],
                 l.position, maze_state_text());
        if (httpd_queue_work(server_handle, ws_send_work, msg) != ESP_OK)
            free(msg);
    }
}

static esp_err_t start_tuning_http_server_private()
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_LOGI(TAG, "Starting HTTP Server");
    if (httpd_start(&server, &config) != ESP_OK)
    {
        ESP_LOGE(TAG, "start server failed");
        return ESP_FAIL;
    }
    
    httpd_uri_t tuning_pid_post_uri = {
        .uri = "/api/v1/pid",
        .method = HTTP_POST,
        .handler = tuning_pid_post_handler,
        .user_ctx = NULL
    };
    if (httpd_register_uri_handler(server, &tuning_pid_post_uri) != ESP_OK)
    {
        ESP_LOGE(TAG, "register post uri failed");
        return ESP_FAIL;
    }

    // Before the "/*" file handler, which would otherwise catch these URLs
    httpd_uri_t lsa_ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = lsa_ws_handler,
        .user_ctx = NULL,
        .is_websocket = true
    };
    if (httpd_register_uri_handler(server, &lsa_ws_uri) != ESP_OK)
    {
        ESP_LOGE(TAG, "register ws uri failed");
        return ESP_FAIL;
    }

    httpd_uri_t lsa_get_uri = {
        .uri = "/api/v1/lsa",
        .method = HTTP_GET,
        .handler = lsa_get_handler,
        .user_ctx = NULL
    };
    if (httpd_register_uri_handler(server, &lsa_get_uri) != ESP_OK)
    {
        ESP_LOGE(TAG, "register lsa uri failed");
        return ESP_FAIL;
    }

    httpd_uri_t common_get_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = rest_common_get_handler,
        .user_ctx = NULL
    };
    if(httpd_register_uri_handler(server, &common_get_uri) != ESP_OK)
    {
        ESP_LOGE(TAG, "register get uri failed");
        return ESP_FAIL;
    }

    server_handle = server;
    xTaskCreate(&lsa_push_task, "lsa_push", 3072, NULL, 1, NULL);
    return ESP_OK;
}

pid_const_t read_pid_const()
{
    return pid_constants;
}

void reset_val_changed_pid_const()
{
    pid_constants.val_changed = false;
}

void start_tuning_http_server()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    initialise_mdns();
    netbiosns_init();
    netbiosns_set_name(MDNS_HOST_NAME);

    connect_to_wifi();
    ESP_ERROR_CHECK(init_fs());
    ESP_ERROR_CHECK(start_tuning_http_server_private());
}
