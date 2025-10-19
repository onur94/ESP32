#include <string.h>
#include <stdio.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_app_desc.h"

#define WIFI_SSID "WIFI_SSID"
#define WIFI_PASS "WIFI_PASS"

static const char *TAG = "OTA_WEB";
static esp_ota_handle_t ota_handle = 0;
static const esp_partition_t *update_partition = NULL;
int retry_num = 0;

static const char *index_html =
"<!doctype html><html><head><meta charset='utf-8'/>"
"<meta name='viewport' content='width=device-width,initial-scale=1'/>"
"<title>ESP32 OTA</title>"
"<style>"
"body{font-family:Segoe UI,Roboto,sans-serif;background:#0d1117;color:#c9d1d9;margin:0}"
".card{max-width:700px;margin:60px auto;background:#161b22;padding:24px;border-radius:12px;box-shadow:0 6px 20px rgba(0,0,0,.3)}"
"h1{margin-top:0;font-size:22px;color:#58a6ff}"
".info{margin:10px 0 18px;line-height:1.6}"
"button{background:#238636;border:0;color:#fff;padding:10px 18px;border-radius:8px;font-weight:600;cursor:pointer}"
"button:disabled{opacity:.5;cursor:not-allowed}"
"input[type=file]{display:block;margin:14px 0 18px;width:100%}"
".bar{height:10px;background:#30363d;border-radius:6px;overflow:hidden}"
".bar>div{height:100%;width:0;background:#58a6ff;transition:width .2s}"
".status{margin-top:12px;font-weight:600}"
".ok{color:#2ea043}.err{color:#f85149}"
"</style></head><body><div class='card'>"
"<h1>ESP32 OTA Güncelleme</h1>"
"<div class='info'>"
"<div><b>Cihaz:</b> <span id='chip'>—</span></div>"
"<div><b>Çalışan Partition:</b> <span id='part'>—</span></div>"
"<div><b>Sürüm:</b> <span id='ver'>—</span></div>"
"</div>"
"<form id='f' onsubmit='return false;'>"
"<input id='file' name='firmware' type='file' accept='.bin' required />"
"<button id='btn' type='button'>Yükle ve Güncelle</button>"
"</form>"
"<div class='bar'><div id='prog'></div></div>"
"<div class='status' id='st'></div>"
"<script>"
"async function info(){try{const r=await fetch('/api/info');if(r.ok){const j=await r.json();"
"document.getElementById('chip').textContent=j.chip||'-';"
"document.getElementById('part').textContent=j.running_partition||'-';"
"document.getElementById('ver').textContent=j.app_ver||'-';"
"}}catch(e){}}info();"
"const file=document.getElementById('file'),btn=document.getElementById('btn'),bar=document.getElementById('prog'),st=document.getElementById('st');"
"function setStatus(t,ok){st.textContent=t;st.className='status '+(ok?'ok':'err');}"
"btn.addEventListener('click',()=>{"
" if(!file.files.length){setStatus('Lütfen .bin dosyası seçin',false);return;}"
" const fd=new FormData();fd.append('firmware',file.files[0],file.files[0].name);"
" const xhr=new XMLHttpRequest();xhr.open('POST','/api/ota');"
" btn.disabled=true;setStatus('Yükleniyor...',true);"
" xhr.upload.onprogress=e=>{if(e.lengthComputable){bar.style.width=Math.round(e.loaded*100/e.total)+'%';}};"
" xhr.onload=()=>{btn.disabled=false;if(xhr.status===200){try{const j=JSON.parse(xhr.responseText);if(j.ok){setStatus('Güncelleme tamamlandı, cihaz yeniden başlatılıyor...',true);bar.style.width='100%';setTimeout(()=>location.reload(),8000);}else setStatus('Hata: '+(j.error||'Bilinmeyen'),false);}catch(_){setStatus('Cihaz yeniden başlatılıyor...',true);}}else setStatus('HTTP '+xhr.status+' hata',false);};"
" xhr.onerror=()=>{btn.disabled=false;setStatus('Bağlantı hatası',false);};"
" xhr.send(fd);"
"});"
"</script></div></body></html>";

/* ---------- /api/info handler ---------- */
static esp_err_t h_info(httpd_req_t *req)
{
    esp_chip_info_t chip; esp_chip_info(&chip);
    const esp_app_desc_t *ad = esp_app_get_description();
    const esp_partition_t *run = esp_ota_get_running_partition();
    char json[256];
    snprintf(json, sizeof(json),
        "{ \"chip\": \"ESP32 (%d cores)\", \"app_ver\": \"%s\", \"running_partition\": \"%s\" }",
        chip.cores,
        ad ? ad->version : "-",
        run ? run->label : "-");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

/* ---------- OTA Upload Handler ---------- */
static esp_err_t h_ota(httpd_req_t *req)
{
    char buf[1024]; int received;
    update_partition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGI(TAG, "Writing to partition: %s", update_partition->label);
    ESP_ERROR_CHECK(esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle));

    bool started = false;
    while ((received = httpd_req_recv(req, buf, sizeof(buf))) > 0) {
        if (!started) {
            char *body_start = strstr(buf, "\r\n\r\n");
            if (body_start) {
                body_start += 4;
                int len = received - (body_start - buf);
                if (len > 0) esp_ota_write(ota_handle, body_start, len);
                started = true;
            }
        } else {
            char *b = strstr(buf, "------WebKitFormBoundary");
            if (b) {
                int len = b - buf - 2;
                if (len > 0) esp_ota_write(ota_handle, buf, len);
                break;
            }
            esp_ota_write(ota_handle, buf, received);
        }
    }

    esp_err_t err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ota_end fail");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"ota_end\"}");
        return ESP_OK;
    }
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "boot_partition fail");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"boot_partition\"}");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "OTA OK. Rebooting...");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();
    return ESP_OK;
}

/* ---------- /index ---------- */
static esp_err_t h_index(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN ? strlen(index_html) : 0);
    return ESP_OK;
}

/* ---------- HTTP server ---------- */
static void start_httpd(void)
{
    httpd_handle_t srv = NULL;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192;
    cfg.lru_purge_enable = true;
    ESP_ERROR_CHECK(httpd_start(&srv, &cfg));

    httpd_register_uri_handler(srv, &(httpd_uri_t){"/", HTTP_GET, h_index, 0});
    httpd_register_uri_handler(srv, &(httpd_uri_t){"/api/info", HTTP_GET, h_info, 0});
    httpd_register_uri_handler(srv, &(httpd_uri_t){"/api/ota", HTTP_POST, h_ota, 0});

    ESP_LOGI(TAG, "HTTP server started");
}

/* ---------- Wi-Fi ---------- */
void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
	if(event_id == WIFI_EVENT_STA_START)
	{
		ESP_LOGI(TAG, "WiFi Connecting...");
		vTaskDelay(pdMS_TO_TICKS(1000));
		esp_wifi_connect();
	}
	else if (event_id == WIFI_EVENT_STA_CONNECTED)
	{
		ESP_LOGI(TAG, "WiFi Connected");
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
	{
		ESP_LOGI(TAG, "WiFi lost connection");
		if(retry_num < 5)
		{
			esp_wifi_connect();
			retry_num++;
			ESP_LOGI(TAG, "Retrying to Connect...");
		}
		else
		{
			ESP_LOGW(TAG, "MCU restarting!");
			esp_restart();
		}
	}
	else if (event_id == IP_EVENT_STA_GOT_IP)
	{
		ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		char wifiIPstr[64];
		sprintf(wifiIPstr, IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Wifi got ip: %s", wifiIPstr);
	}
}

void wifi_start()
{
	esp_netif_init();
	esp_event_loop_create_default();
	esp_netif_create_default_wifi_sta();
	wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&wifi_initiation);
	esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL); 
	esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
	wifi_config_t wifi_configuration ={
	.sta= {
			.ssid 		= WIFI_SSID,
			.password	= WIFI_PASS,
		}
	};
	esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
	esp_wifi_start();
	esp_wifi_set_mode(WIFI_MODE_STA);
}

/* ---------- Main ---------- */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    wifi_start();
    vTaskDelay(pdMS_TO_TICKS(2000));
    start_httpd();
    ESP_LOGI(TAG, "OtaExample application");
}
