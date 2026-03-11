# ESP32-S3 DAQ: System Akwizycji Danych Silników BLDC
System akwizycji danych (DAQ) o wysokiej rozdzielczości oparty na mikrokontrolerze **ESP32-S3** oraz 24-bitowym przetworniku ADC **MCP3564RT**. Projekt został zaprojektowany w celu precyzyjnego monitorowania parametrów prądowych silników BLDC i rejestracji danych na karcie microSD do późniejszej analizy (np. pod kątem uczenia maszynowego).
---
## 1. Kluczowe cechy
* **Wielokanałowość:** Jednoczesny pomiar z 4 układów INA240 monitorujących fazy silnika.
* **System czasu rzeczywistego:** Implementacja w oparciu o FreeRTOS – pełna separacja zadań pomiarowych (akwizycji) od procesu zapisu na nośnik danych.
* **Kalibracja programowa:** Wbudowany algorytm automatycznego usuwania offsetu dla każdego kanału pomiarowego.
* **Składowanie danych:** Stabilny zapis w formacie `.csv` na karcie microSD z wykorzystaniem systemu plików FATFS.
---
## 2. Specyfikacja techniczna
### Warstwa sprzętowa (Hardware)
* **MCU:** ESP32-S3 (Dual-core, AI Acceleration support).
* **ADC:** MCP3564RT-E/ST (24-bit Delta-Sigma, do 153.6 ksps).
* **Wzmacniacze pomiarowe:** 4x INA240 (Gain 20 V/V, wysokie tłumienie PWM).
* **Boczniki prądowe:** 3 mΩ.
* **Interfejsy komunikacyjne:** 
  *  `SPI2 (FSPI)`: Dedykowany dla ADC (częstotliwość 20 MHz).
  *  `SPI3 (SUBSPI)`: Dedykowany dla karty microSD.

### Konfiguracja Pinów (Pinout)
#### **Magistrala SPI2 — Przetwornik ADC (MCP3564)**
* **SCK / MISO / MOSI:** `12 / 13 / 11`
* **Chip Select (CS):** `10`
* **Interrupt (IRQ):** `4`

#### **Magistrala SPI3 — Karta microSD**
* **SCK / MISO / MOSI:** `36 / 37 / 35`
* **Chip Select (CS):** `38`
---
## 3. Struktura projektu
* `main.c`: Główna logika aplikacji, zarządzanie zadaniami FreeRTOS, inicjalizacja karty SD i orkiestracja przepływu danych.
* `mcp3564_driver.c/h`: Niskopoziomowy sterownik ADC, bezpośrednia obsługa rejestrów układu oraz przeliczenia wartości fizycznych.
* `CMakeLists.txt`: Konfiguracja systemu budowania w środowisku PlatformIO.
---
## 4. O projekcie
Projekt stanowi część pracy dyplomowej **" ZASTOSOWANIE WYBRANYCH METOD UCZENIA MASZYNOWEGO DO WYKRYWANIA USZKODZEŃ NAPĘDU ELEKTRYCZNEGO DRONA".**

**Autor:** Pavel Tshonek

**Rok:** 2026


