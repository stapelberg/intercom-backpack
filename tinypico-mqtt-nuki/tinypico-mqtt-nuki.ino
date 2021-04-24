#include <PubSubClient.h>
#include <TinyPICO.h>
#include <WiFi.h>
#include <driver/adc.h>
#include <driver/dac.h>

#include "scsfilter.h"

#define GPIO_DOOR_OPEN 27
#define GPIO_NUKI_YELLOW 14
#define GPIO_NUKI_BLUE 15
#define GPIO_FLOOR_RING 4

void nukiRing(void *pvParameters);

WiFiClient wificlient;
PubSubClient client(wificlient);

// TODO: give this a constructor or something
scsfilter sf;

static bool ignore_scs_telegrams = false;
static uint8_t ignore_scs_checksum;
static TimerHandle_t xQuietTimer;

void scsquietnow(void *pvParameters) {
  ignore_scs_telegrams = false;
  Serial.println("timer fire, messages okay again");
  xTimerDelete(xQuietTimer, 10);
  xQuietTimer = NULL;
}

void taskscsprocess(void *pvParameters) {
  for (;;) {
    if (Serial2.available() == 0) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    byte incomingByte = Serial2.read();
    Serial.printf("[core %d] SCSRX: ", xPortGetCoreID());
    Serial.println(incomingByte, HEX);
    client.publish("doorbell/debug/scsrx", &incomingByte, 1);

    // We store the checksum of telegrams to ignore, so that different
    // messages are not accidentally ignored by the quiet timer.
    //
    // This can happen when e.g. the mailman rings the neighbor first, and then
    // my doorbell: the ring signal for my doorbell was ignored while the bus
    // was still active from delivering the neighbor ring signal.
    const bool ignore_this_telegram = ignore_scs_telegrams;
    const uint8_t ignore_this_telegram_checksum = ignore_scs_checksum;

    if (xQuietTimer != NULL) {
      xTimerReset(xQuietTimer, 10);
    }

    // feed this byte into the SCS decoder:
    sf_WriteByte(&sf, incomingByte);

    if (!sf_completeAndValid(&sf)) {
      continue;
    }

    const uint8_t checksum = sf.tbuf[5];
    Serial.printf("complete SCS telegram detected, checksum %x\n", checksum);

    if (ignore_this_telegram && ignore_this_telegram_checksum == checksum) {
      Serial.println("ignoring retransmission");
      continue;
    }

    {
      // Ignore the SCS bus retransmissions by waiting until the bus quiets
      // down: this timer gets restarted on every received byte.
      ignore_scs_telegrams = true;
      ignore_scs_checksum = checksum;
      xQuietTimer =
          xTimerCreate("scsquiet", // human readable text name for debugging
                       pdMS_TO_TICKS(50), // xTimerPeriod (in ticks)
                       pdFALSE,           // uxAutoReload (or one-shot)
                       NULL,              // timer id
                       scsquietnow);      // callback function
      if (xQuietTimer != NULL) {
        xTimerStart(xQuietTimer, 0);
        Serial.println("timer start");
      }
    }

    // Publish an MQTT message for each SCS telegram:
    client.publish("doorbell/events/scs", sf.tbuf, telegramLen);
    Serial.println("mqtt publish");

    if (sf_ringForApartment(&sf) == 3) {
      client.publish("doorbell/events/ring", "house");
      Serial.println("ring detected, triggering nuki opener");
      xTaskCreatePinnedToCore(nukiRing, "nukiRing", 2048, NULL, 1, NULL,
                              PRO_CPU_NUM);
    }
  }
}

// --------------------------------------------------------------------------------

void connectToWiFi(void) {
  Serial.println("WiFi: configuring");
  WiFi.mode(WIFI_STA);
  // required to set hostname properly:
  // https://github.com/espressif/arduino-esp32/issues/3438#issuecomment-721428310
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname("doorbelltp");
  WiFi.begin("secret", "secret");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi: connecting...");
    delay(100);
  }
  Serial.print("WiFi: connected: mac=");
  Serial.print(WiFi.macAddress());
  Serial.print(" ip=");
  Serial.print(WiFi.localIP());
  Serial.println("");
}

// --------------------------------------------------------------------------------

void nukiRing(void *pvParameters) {
  Serial.printf("[core %d] nukiRing\n", xPortGetCoreID());
  digitalWrite(GPIO_NUKI_YELLOW, HIGH);
  vTaskDelay(pdMS_TO_TICKS(1000));
  digitalWrite(GPIO_NUKI_YELLOW, LOW);
  client.publish("doorbell/debug/nukiring", "ring");
  vTaskDelete(NULL);
}

void doorUnlock(void *pvParameters) {
  Serial.printf("[core %d] doorUnlock\n", xPortGetCoreID());
  digitalWrite(GPIO_DOOR_OPEN, LOW);
  vTaskDelay(pdMS_TO_TICKS(500));
  digitalWrite(GPIO_DOOR_OPEN, HIGH);
  client.publish("doorbell/debug/doorunlock", "doorunlock");
  vTaskDelete(NULL);
}

SemaphoreHandle_t xNukiOpenSemaphore = NULL;

// IRAM_ATTR: all interrupt service routines must go into the internal SRAM:
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/general-notes.html#iram-instruction-ram
void IRAM_ATTR nukiOpen() { xSemaphoreGiveFromISR(xNukiOpenSemaphore, NULL); }

// setupnuki0 is the part of the setup routine that needs to run on core 0, so
// that the interrupt handler will also run on core 0:
// https://rntlab.com/question/esp32-designating-a-specific-core-for-specific-interrupt-service/
void setupnuki0(void *pvParameters) {
  attachInterrupt(GPIO_NUKI_BLUE, nukiOpen, FALLING);
  for (;;) {
    if (xSemaphoreTake(xNukiOpenSemaphore, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    // TODO: where do these come from? unclear.
    if (digitalRead(GPIO_NUKI_BLUE) != LOW) {
      Serial.printf("ignoring spurious nukiOpen\n");
      continue;
    }

    xTaskCreatePinnedToCore(doorUnlock, "doorUnlock", 2048, NULL, 1, NULL,
                            PRO_CPU_NUM);
  }
}

void setupnuki(void) {
  xNukiOpenSemaphore = xSemaphoreCreateBinary();

  // pin GPIO_NUKI_YELLOW: connected to nuki opener yellow cable (RING)
  pinMode(GPIO_NUKI_YELLOW, OUTPUT);
  digitalWrite(GPIO_NUKI_YELLOW, LOW);

  // pin GPIO_NUKI_BLUE: connected to nuki opener blue cable (OPEN)
  pinMode(GPIO_NUKI_BLUE, INPUT_PULLUP);
  xTaskCreatePinnedToCore(setupnuki0, "setupnuki0", 2048, NULL, 1, NULL,
                          PRO_CPU_NUM);
}

// --------------------------------------------------------------------------------

void floorRingNuki(void *pvParameters) {
  Serial.printf("[core %d] floorRing\n", xPortGetCoreID());
  client.publish("doorbell/events/ring", "floor");
  const byte state = digitalRead(GPIO_FLOOR_RING);
  digitalWrite(GPIO_NUKI_YELLOW, HIGH);
  vTaskDelay(pdMS_TO_TICKS(1000));
  digitalWrite(GPIO_NUKI_YELLOW, LOW);
  client.publish("doorbell/debug/floorring", state == LOW ? "low" : "high");
  vTaskDelete(NULL);
}

SemaphoreHandle_t xFloorOpenSemaphore = NULL;

// IRAM_ATTR: all interrupt service routines must go into the internal SRAM:
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/general-notes.html#iram-instruction-ram
void IRAM_ATTR floorRing() { xSemaphoreGiveFromISR(xFloorOpenSemaphore, NULL); }

static byte floorRingState = LOW;
static unsigned long lastFloorRing;

// setupfloor0 is the part of the setup routine that needs to run on core 0, so
// that the interrupt handler will also run on core 0:
// https://rntlab.com/question/esp32-designating-a-specific-core-for-specific-interrupt-service/
void setupfloor0(void *pvParameters) {
  attachInterrupt(GPIO_FLOOR_RING, floorRing, RISING);
  for (;;) {
    if (xSemaphoreTake(xFloorOpenSemaphore, portMAX_DELAY) != pdTRUE) {
      continue;
    }
    if (floorRingState == LOW && (millis() - lastFloorRing) < 1000) {
      continue; // debounce by waiting it out
    }

    const byte state = digitalRead(GPIO_FLOOR_RING);
    Serial.printf("GPIO_FLOOR_RING was %s\n",
                  floorRingState == LOW ? "low" : "high");
    Serial.printf("GPIO_FLOOR_RING is %s\n", state == LOW ? "low" : "high");
    Serial.printf("\n");
    if (floorRingState == LOW && state == HIGH) {
      // debounce: wait if HIGH after 100ms still
      vTaskDelay(pdMS_TO_TICKS(100));
      const byte debouncedstate = digitalRead(GPIO_FLOOR_RING);
      Serial.printf("debouncedstate is %s\n",
                    debouncedstate == LOW ? "low" : "high");
      if (debouncedstate != HIGH) {
        continue; // too short a press, or spurious
      }
      xTaskCreatePinnedToCore(floorRingNuki, "floorRingNuki", 2048, NULL, 1,
                              NULL, PRO_CPU_NUM);
      lastFloorRing = millis();
    }
    floorRingState = state;
  }
}

void setupfloor(void) {
  xFloorOpenSemaphore = xSemaphoreCreateBinary();

  // pin GPIO_FLOOR_RING: connected to floor ring signal
  pinMode(GPIO_FLOOR_RING, INPUT_PULLDOWN);
  xTaskCreatePinnedToCore(setupfloor0, "setupfloor0", 2048, NULL, 1, NULL,
                          PRO_CPU_NUM);
}

// --------------------------------------------------------------------------------

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (strcmp(topic, "doorbell/cmd/unlock") == 0) {
    xTaskCreatePinnedToCore(doorUnlock, "doorUnlock", 2048, NULL, 1, NULL,
                            PRO_CPU_NUM);
  }

  if (strcmp(topic, "doorbell/debug/cmd/ring") == 0) {
    xTaskCreatePinnedToCore(nukiRing, "nukiRing", 2048, NULL, 1, NULL,
                            PRO_CPU_NUM);
  }
}

void taskmqtt(void *pvParameters) {
  for (;;) {
    if (!client.connected()) {
      // TODO: do we need a timeout here? does client.connect only block the
      // taskmqtt FreeRTOS task, or all FreeRTOS tasks on this core?
      client.connect("doorbelltp" /* clientid */);
      client.publish("doorbell/presence", "fresh from logon");
      client.subscribe("doorbell/cmd/unlock");
      client.subscribe("doorbell/debug/cmd/ring");
    }

    // Poll PubSubClient for new messages and invoke the callback.
    // Should be called as infrequent as one is willing to delay
    // reacting to MQTT messages.
    // Should not be called too frequently to avoid strain on
    // the network hardware:
    // https://github.com/knolleary/pubsubclient/issues/756#issuecomment-654335096
    client.loop();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// --------------------------------------------------------------------------------

void taskalive(void *pvParameters) {
  for (;;) {
    Serial.printf("still running\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// --------------------------------------------------------------------------------

void taskreconnect(void *pvParameters) {
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.printf("reconnecting to WiFi\n");
      connectToWiFi();
    }
    vTaskDelay(pdMS_TO_TICKS(60 * 1000));
  }
}

// --------------------------------------------------------------------------------

TinyPICO tp = TinyPICO();

void setup() {
  // Turn on Power LED:
  tp.DotStar_SetPixelColor(255, 128, 0);

  memset(&sf, '\0', sizeof(scsfilter));

  Serial.begin(115200);
  delay(1000);

  connectToWiFi();

  client.setServer("dr.lan", 1883);
  client.setCallback(callback);

  pinMode(GPIO_DOOR_OPEN, OUTPUT);
  digitalWrite(GPIO_DOOR_OPEN, HIGH);

  setupnuki();
  setupfloor();

  // setup UART2 (U2UXD) on GPIO pin 25 (RX)
  Serial2.begin(9600, SERIAL_8N1, 25, 26);

  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/freertos-smp.html#scheduling
  // CPU 0 (PRO_CPU): protocol cpu
  // CPU 1 (APP_CPU): application cpu
  //
  // arduino-esp32 defaults to using APP_CPU for loop() and WiFi onEvent
  // callbacks. We leave loop() empty and use APP_CPU entirely for SCS ADC
  // conversion:
  disableCore1WDT();

  xTaskCreatePinnedToCore(taskscsprocess, "SCS", 2048, NULL, 1, NULL,
                          PRO_CPU_NUM);

  xTaskCreatePinnedToCore(taskmqtt, "MQTT", 2048, NULL, 1, NULL, PRO_CPU_NUM);

  xTaskCreatePinnedToCore(taskalive, "Alive", 2048, NULL, 1, NULL, PRO_CPU_NUM);

  xTaskCreatePinnedToCore(taskreconnect, "reconnect", 2048, NULL, 1, NULL,
                          PRO_CPU_NUM);
}

// Arduino loop function unused in favor of FreeRTOS tasks.
// The ESP32 Arduino Board Support Package runs loop in a FreeRTOS task itself.
void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
