/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <rc522.h>		//Biblioteka do modułu RFID rc522
#include <FLASH_PAGE.h> //Biblioteka do zapisu/odczytu flash
#include <MY_NRF24.h>	//Biblioteka do modułu radiowego
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*---------------------------------------ZMIENNE---------------------------------------------------*/


/*
Rozkład pinów dla projektu:
PIN:			ETYKIETA:			TRYB PRACY:				FUNKCJONALNOŚĆ:

PC0				NRF_CE				GPIO_OUTPUT				Wysyłanie danych do nRF24
PC3				NRF_CSN				GPIO_OUTPUT				Wysyłanie danych do nRF24
PC13			PN					GPIO_EXTI3				Przycisk - programowanie flash (low - on, high - off)
PA2				USART_NAD			USART2_TX				Nadawanie poprzez USART
PA3				USART_ODB			USART2_RX				Odbieranie poprzez USART
PA5				LDG					GPIO_OUTPUT				Zielona dioda (low - off, high - on)
PB11			RES					GPIO_OUTPUT				Ustawia tryb komunikacji RC522
PB12			SPI_CS				GPIO_OUTPUT				Komunikacja z RC522
PB13			SCK					SPI2_SCK				Komunikacja z RC522
PB14			MISO				SPI2_MISO				Komunikacja z RC522
PB15			MOSI				SPI2_MOSI				Komunikacja z RC522
PC10			NRF_SCK				SPI3_SCK				Komunikacja z nRF24
PB4				NRF_MISO			SPI3_MISO				Komunikacja z nRF24
PB5				NRF_MOSI			SPI3_MOSI				Komunikacja z nRF24
PB7				RELAY_OUT			GPIO_OUTPUT				Sterowanie stanem przekaźnika (high - on, low - off)
PB8				NRF_IRQ				GPIO_EXTI8				Jeszcze brak, docelowo przerwania
PB9				NRF_VCC				GP IO_OUTPUT				Zasilanie nRF24 (high - on, low - off)
PA0				SPEAKER				GPIO_OUTPUT				Sygnał głośnika
*/



//--------Definicja SPI dla RC522
#define cs_reset() 					HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET)
#define cs_set() 					HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET)
//Koniec: Definicja SPI dla RC522


//Bufory kołowe do komunikacji poprzez USART
#define USART_NAD_BUF_LEN 1512				//Rozmiar buforu nadawczego
#define USART_ODB_BUF_LEN 1512				//Rozmiar buforU odbiorczego
uint8_t USART_NADBuf[USART_NAD_BUF_LEN];	//Bufor nadawczy
uint8_t USART_ODBBuf[USART_ODB_BUF_LEN];	//Bufor odbiorczy
uint16_t USART_NAD_Empty=0;					//Pozycja dla pustego miejsca w buf. nadawczym
uint16_t USART_NAD_Busy=0;					//Pozycja dla zajętego miejsca w buf. nadawczym
uint16_t USART_ODB_Empty=0;					//Pozycja dla pustego miejsca w buf. odbiorczym
uint16_t USART_ODB_Busy=0;					//Pozycja dla zajętego miejsca w buf. odbiorczym
//Koniec: Bufory kołowe do komunikacji poprzez USART


//Liczniki, stany oraz zmienne sprawdzające
uint8_t Licznik_LDG_Czest = 0;		//Licznik częstotliwości migania diody przy odczycie RFID
uint16_t Licznik_LDG_Dlugosc = 0;	//Licznik migania diody po odczycie RFID
uint8_t Opoznienie = 1;				//Zmienna przełączająca - decyduje, czy opóźnienie jest włączone, czy nie
uint16_t Licznik_Opoznienie = 0;	//Licznik dla opóźnienia do zapisu/odczyty RFID
uint16_t Licznik_ID_Zapis = 0;		//Licznik do zapisu RFID (Zegar: IDSaving)
uint16_t Licznik = 0; 				//Licznik ogólny
uint8_t Door_Open = 0;				//zmienna informująca, czy furtka jest otwarta
uint8_t Door_PermOpen = 0;			//tryb stałego otwarcia furtki (0 - wył)
uint8_t PN_Licznik_Toggle = 0;		//Włączenie/wyłączenie licznika przycisku
uint32_t PN_OFF_Licznik = 0;		//Licznik nie naciśniętego przycisku
uint32_t PN_OFF_Licznik_Zapisany=0;	//zapis licznika do pamięci
uint32_t PN_ON_Licznik = 0;			//Licznik wciśniętego przycisku
uint32_t PN_ON_Licznik_Zapisany = 0;//zapis licznika do pamięci
uint8_t PN_Verify1 = 0;				//Programowanie flash
uint8_t PN_Verify2 = 0;				//Programowanie flash
int i = 0;							//Licznik do pętli
int g = 0;							//zmienna do tablic z usarta
uint16_t LicznikPerm = 0;			//Licznik do resetowania diody przy otwartej furtce
uint16_t Licznik_Ramka = 0;			//Licznik dla timeoutu ramki
uint8_t ramkaReset = 0;				//Zmienna przełączająca timeout dla ramki
uint8_t dzwiekPrzelacznik = 1;		//Dzwiek wl/wyl
uint8_t przekroczonoZakres = 0;		//Sterowanie przerkoczeniem zakresu w ramce komunikacji
uint8_t bladKodowania = 0;			//Sterowanie bledem kodowania (znak niekodowany po \)
uint8_t trigger_flashClear = 0;		//przełącznik czyszczenia flash
uint16_t licznik_flashClear = 0;	//licznik czyszczenia flash
//Koniec: Liczniki, stany oraz zmienne sprawdzające


//Zmienne decydujące o czasie trwania akcji
uint16_t Czas_OtwarciaFurtki = 5000;			//Czas otwarcia furtki po otwarciu RFID lub sygnale krótkim
uint16_t Czas_Opoznienie = 6000;				//Czas opóźnienia akcji
uint16_t Czas_ZapisuFlash = 6000;				//Czas, w którym można zapisać do flash po włączeniu trybu
uint16_t Czas_InterwalyResetu = 8000;			//Interwały dla resetu diody nadajnika
uint16_t Czas_WyczyszczenieFlash = 20000;		//Czas wciśnięcia przycisku do zresetowania flash
uint16_t Czas_ZapisPrzycisk_E1D = 2000;			//Zakres czasu dolny dla etapu 1 (E1D) sekwencji zapisu flash w ms (od 2s..)
uint16_t Czas_ZapisPrzycisk_E1G = 4000;			//(..do 4s)
uint16_t Czas_ZapisPrzycisk_E2D = 3000;			// od 3s...
uint16_t Czas_ZapisPrzycisk_E2G = 6000;			// ...do 6s
uint16_t Czas_ZapisPrzycisk_E3D = 5000;			//od 5s...
uint16_t Czas_ZapisPrzycisk_E3G = 10000;		//...do 10s
uint16_t Czas_ZapisPrzycisk_Zerowanie = 10000;	//Interwały zerowania licznika off przycisku i stanów weryfikacji sekwencji zapisu flash
uint16_t Czas_OdczytFlash_MiganieDiody = 2000;	//Czas migania diody po poprawnym odczycie RFID
uint16_t Czas_OdczytFlash_Czestotliwosc = 250;	//Częstotliwość migania diody po odczycie RFID
uint16_t Czas_resetowaniaRamki = 4500;			//Timeout dla komunikacji

/*PAMIEĆ FLASH: Info
Zakres pamięci: 0x0800 0000 -  0x0807 FFFF
Wybrany zakres pamięci Flash dla RFID: 0x08032010 do 0x0807D010
Tagi zapisywane są co 4 bajty, co daje 76 800 miejsc dla tagów
Czyszczenie:
Więc od 0x08032010 do 0x0807D010 co 2048b, więc co 0x800 HEX jeden tag
Wywołanie zapisu do flash:
2 - 4s Przycisk wciśnięty,
3 - 6s Przycisk nie naciśnięty,
5 - 20s Przycisk wciśnięty
Wywołanie czyszczenia flash:
>20s Przycisk wciśnięty
Koniec: Flash Info*/

//RC522 - Zmienne
uint32_t ID[4];						//Kontener dla ID karty
uint8_t ID_Weryfikacja;				//Zmienna informująca, czy ID się zgadza z zapisem w pamięci
uint8_t ID_Zapis = 0; 				//Zmienna włączająca w tryb zapisu flash
//Koniec: RC522 - Zmienne


//nRF24 - Zmienne
uint64_t AdresOdbiorczy = 0x11223344AA;	//Adres odbiorczy modułu radiowego
uint64_t AdresNadawczy = 0x12223344AA;	//Adres nadawczy modułu radiowego
char klucz[50];							//Kontener na przychodzące dane
uint32_t kluczKrotki = 391020153;		//Klucz dla otwarcia furtki na określony czas
uint32_t kluczDlugi = 594201501;		//Klucz dla otwarcia/zamknięcia furtki na stałe
static char resetDiody[32] = "325112";	//Włącza diodę w nadajniku, jeśli furtka otwarta jest na stałe
static char resetDiody2[32] = "435345";	//Wyłącza diodę w nadajniku, jeśli furtka jest zamknięta
uint8_t resetDiody_wysylka = 0;			//Zmienna decydująca
//Koniec: nRF24 - Zmienne


//Zmienne głośnika
uint16_t Tony[] = {1911,1702,1516,1431,1275,1136,1012}; //Wartości opóźnienia w mikrosekundach
//od najmniejszej częstotliwości do największej
//Koniec: Zmienne głośnika


//Komendy
static char komendaOtworz[248] = "OPEN";				//Otwiera elektrozaczep
static char komendaOtworzStale[248] = "OPENPERM";		//Otwiera stale elektrozaczep
static char komendaZamknij[248] = "CLOSE";				//Zamyka elektrozaczep
static char komendaFlashZapisz[248] = "FLASHSAVE";		//Zapis RFID do flash
static char komendaFlashWyczysc[248] = "FLASHCLEAR";	//Czyszczenie flash
static char komendaZmienCzas[248] = "CHANGETIMEOPEN:";	//Zmiana czasu otwarcia elektrozaczepu
static char komendaDzwiek[248] = "SOUND";				//WL/WYL dzwiek
//Koniec komend


//Zmienne komunikacji UART
uint8_t znak;							//Zmienna przechowująca aktualny znak z buforu odbiorczego
uint8_t odbiorRamki = 0;				//Zmienna informująca, czy trwa odbiór właściwej części ramki
uint8_t poczatekRamki = 0;				//Zmienna informująca, czy odebrano pierwszy znak
uint8_t koniecRamki = 0;				//Zmienna informująca, czy odebrano ostatni znak
uint8_t errorZabronionyZnak = 0;		//Zmienna przełączająca dla błędu
uint16_t ramkaIDX = 0;					//Indeks ramki
uint8_t ramka[256];						//Zmienna przechwytująca ramkę komunikacji
uint8_t dlugoscRamki = 0;				//Zmienna przechowująca długość ramki
uint8_t czyszczenieFlash = 0;			//Zmienna przełączająca dla czyszczenia flash
uint8_t komendaZmienCzasTrigger = 0;	//Zmienna informująca o przypadku komendy zmiany czasu
uint8_t zmianaCzasuTablica[5];			//Zmienna przechwytująca czas do komendy
uint8_t zmianaCzasuError = 0;			//Zmienna przełączająca dla błędu
uint16_t tempCzas;						//Zmienna tymczasowa do konwersji czasu
char Odbiorca[4] = "NCL";				//Nazwa odbiorcy
char OdbiorcaSpr[4];					//Zmienna do sprawdzenia, czy komunikat jest do płytki
char Nadawca[4];						//Zmienna przechwytująca nadawcę
char komenda[248];						//Zmienna przechwytująca komendę w ramce
uint8_t kodowanie = 0;
//Koniec zmiennych komunikacji UART




/*---------------------------------KONIEC ZMIENNYCH-----------------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/*--------------------------------------FUNKCJE----------------------------------------------*/


 //RC522 - Deklaracje funkcji
uint8_t MFRC522_Request(uint8_t reqMode, uint8_t* TagType); //Odbieranie tagu RFID
uint8_t MFRC522_Anticoll(uint8_t* serNum);					//Anty-kolizja dla RFID
void MFRC522_Init(void);									//Inicjalizacja RC522
//Koniec: RC522 - Deklaracje funkcji



void Zegar(){//----------Początek: Zegar-----------



	if(!Opoznienie){ //Opóźnienie dla odczytu RFID
		Licznik_Opoznienie++;
		if(Licznik_Opoznienie>=Czas_Opoznienie){
			Licznik_Opoznienie = 0;
			Opoznienie = 1;
		}
	}//Koniec: Opóźnienie dla odczytu RFID

	if(Door_Open){ //Oczekiwanie i zamknięcie furtki
		Licznik++;
		if(Licznik>=Czas_OtwarciaFurtki){
			Door_ChangeState();
			if(dzwiekPrzelacznik)
				dzwiek_doorClose();
			Licznik = 0;
			Door_Open = 0;
		}
	}//Koniec: Oczekiwanie i zamknięcie furtki

	if(PN_Licznik_Toggle)//Liczenie czasu stanu przycisku
		PN_ON_Licznik++;
	else
		PN_OFF_Licznik++;//Koniec: Liczenie czasu stanu przycisku

	if(ID_Zapis){//Zapis RFID do pamięci flash - licznik i dioda
		Licznik_ID_Zapis++;
		Licznik++;
		if(Licznik>=500){
			LDG_Toggle();
			Licznik = 0;
		}
		if(Licznik_ID_Zapis>=Czas_ZapisuFlash){
			ID_Zapis = 0;
			Licznik_ID_Zapis = 0;
			HAL_GPIO_WritePin(LDG_GPIO_Port, LDG_Pin, GPIO_PIN_RESET);
			Opoznienie = 0;
			dzwiek_ID_Check_NotOK();
		}
	}//Koniec: Zapis RFID do pamięci flash - licznik i dioda

	if(ID_Weryfikacja){//Poprawny odczyt RFID - miganie diody
		Licznik_LDG_Dlugosc++;
		Licznik_LDG_Czest++;
		if(Licznik_LDG_Czest>=Czas_OdczytFlash_Czestotliwosc){
			LDG_Toggle();
			Licznik_LDG_Czest = 0;
		}
		if(Licznik_LDG_Dlugosc>Czas_OdczytFlash_MiganieDiody){
			Licznik_LDG_Dlugosc = 0;
			ID_Weryfikacja = 0;
		}
		HAL_GPIO_WritePin(LDG_GPIO_Port, LDG_Pin, GPIO_PIN_RESET);
	}//Koniec: Poprawny odczyt RFID - miganie diody


	LicznikPerm++;
	if(LicznikPerm>Czas_InterwalyResetu){
		resetDiody_wysylka = 1;
		LicznikPerm = 0;
	}//Koniec: Reset diody pilota - furtka otwarta


	if(ramkaReset)	//timeout dla ramki
	{
		Licznik_Ramka++;
		if(Licznik_Ramka > Czas_resetowaniaRamki)
			resetRamki();
	} //Koniec: timeout dla ramki


	if(trigger_flashClear){
		licznik_flashClear++;
		if(licznik_flashClear >= 5000){
			licznik_flashClear = 0;
			trigger_flashClear = 0;
		}
	}


}//---------Koniec: Zegar----------



uint8_t USART_ODB_NotEmpty(){
	if(USART_ODB_Empty==USART_ODB_Busy)
		return 0;
	else
		return 1;
}//Koniec: USART_ODB_NotEmpty


uint8_t USART_PobierzZnak(){
	uint8_t temp;
	if(USART_ODB_NotEmpty()){
		temp = USART_ODBBuf[USART_ODB_Busy];
		USART_ODB_Busy++;
		if(USART_ODB_Busy >= USART_ODB_BUF_LEN)
			USART_ODB_Busy = 0;
		return temp;
	}
	else return 0;
}//Koniec: USART_PobierzZnak


uint8_t USART_PobierzLinie(char *buf){
	static uint8_t bf[128];
	static uint8_t idx=0;
	uint8_t ret;
	int x = 0;
	while(USART_ODB_NotEmpty()){
		bf[idx] = USART_PobierzZnak();
		if(((bf[idx]==10)||(bf[idx]==13)))
		{
			bf[idx]=0;
			for(i=0;i<=idx;i++){
				buf[i]=bf[i];
			}
			ret = idx;
			idx = 0;
			return ret;
		}
		else
		{
			idx++;
			if(idx>=128)
				idx = 0;
		}
	}
	return x;
}//Koniec: USART_PobierzLinie


void USART_wysylka(char* format,...){
	char tmp_rs[256];
	uint16_t H;
	__IO int idx;
	va_list arglist;
	va_start(arglist,format);
	vsprintf(tmp_rs,format,arglist);
	va_end(arglist);
	idx = USART_NAD_Empty;
	for(H=0;H<strlen(tmp_rs);H++){
		USART_NADBuf[idx]=tmp_rs[H];
		idx++;
		if(idx >= USART_NAD_BUF_LEN)
			idx = 0;
	}
	__disable_irq();
	if((USART_NAD_Empty==USART_NAD_Busy)&&(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE)==SET))
	{
		USART_NAD_Empty = idx;
		uint8_t tmp = USART_NADBuf[USART_NAD_Busy];
		USART_NAD_Busy++;
		if(USART_NAD_Busy >= USART_NAD_BUF_LEN)
			USART_NAD_Busy = 0;
		HAL_UART_Transmit_IT(&huart2, &tmp, 1);
	}
	else
		USART_NAD_Empty = idx;
	__enable_irq();
}//Koniec: USART_Wysylka


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) //przerwanie do nadawania
{
	if(huart==&huart2)
	{
		if(USART_NAD_Empty!=USART_NAD_Busy)
		{
			uint8_t temp = USART_NADBuf[USART_NAD_Busy];
			USART_NAD_Busy++;
			if(USART_NAD_Busy >= USART_NAD_BUF_LEN)
				USART_NAD_Busy = 0;
			HAL_UART_Transmit_IT(&huart2, &temp, 1);
		}
	}
}//Koniec: Przerwanie do nadawania


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //przerwanie do odbioru
{
	if(huart == &huart2)
	{
		USART_ODB_Empty++;
		if(USART_ODB_Empty >= USART_ODB_BUF_LEN)
			USART_ODB_Empty = 0;
		HAL_UART_Receive_IT(&huart2, &USART_ODBBuf[USART_ODB_Empty], 1);
	}
} //Koniec: Przerwanie do odbioru


void Flash_IDSave(uint32_t ID_IN){
	HAL_FLASH_Unlock();
	__IO uint32_t Flash_check = 0;
	uint32_t AdresZapisu = 0x8032010;
	uint8_t C = 0;
	for(i = 0; i <= 76800; i++){ //wyszukiwanie wolnego miejsca co 4 bajty
		Flash_check = *(__IO uint32_t *)AdresZapisu;
		if(Flash_check == 0xFFFFFFFF){ //znaleziono wolne miejsce
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, AdresZapisu, ID_IN);
			C = 1;
			break;
		}
		else
			AdresZapisu = AdresZapisu + 4;
	}
	if(!C) //nie znaleziono wolnego miejsca - flash dla RFID jest czyszczony
		flashClear();
	HAL_FLASH_Lock();
}


uint8_t Flash_IDCheck(uint32_t ID_IN){//Wyszukanie RFID w pamięci flash
	uint32_t AdresPobrania = 0x8032010;
	__IO uint32_t Flash_check;
	for(i = 0 ; i <= 76800; i++){//Zakres pamięci flash dla RFID to 150 stron
		Flash_check = *(__IO uint32_t *)AdresPobrania;
		if(ID_IN==Flash_check)
			return 1;
		else
			AdresPobrania = AdresPobrania + 4;
	}
	return 0;
}//Koniec: Flash_IDCheck


void LDG_Toggle(){//Przełączenie stanu diody zielonej
	HAL_GPIO_TogglePin(LDG_GPIO_Port, LDG_Pin);
}


void RC522_SetReset(){ //Ustawienie resetu dla RC522, dla komunikacji poprzez SPI
	HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_SET);
	HAL_Delay(5);
}//Koniec: RC522_SetReset


void Door_ChangeState(){//Otworzenie/zamknięcie furtki
	HAL_GPIO_TogglePin(RELAY_OUT_GPIO_Port, RELAY_OUT_Pin);
}//Koniec: Otworzenie/zamknięcie furtki


void delay_us (uint16_t us)//Opóźnienie w mikrosekundach
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}//Koniec: Opóźnienie w mikrosekundach


void dzwiek_ID_Zapis(){//Dźwięk wydawany podczas zapisu karty RFID
	for(uint8_t j = 4; j<=6; j++){
		for(i = 0; i<100; i++){
			HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET);
			delay_us(Tony[j]);
			HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);
			delay_us(Tony[j]);
		}
		HAL_Delay(10);
	}
	for(uint8_t j = 5; j<=6; j++){
		for(i = 0; i<100; i++){
			HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET);
			delay_us(Tony[j]);
			HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);
			delay_us(Tony[j]);
		}
		HAL_Delay(10);
	}
}//Koniec: dzwiek_ID_Zapis


void dzwiek_ID_Check_OK(){//Dźwięk wydawany podczas poprawnego odczytu karty RFID
	for(uint8_t j = 0; j<3; j++){
		for(i = 0; i<40; i++){
			HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET);
			delay_us(Tony[6]);
			HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);
			delay_us(Tony[6]);
		}
		HAL_Delay(50);
	}
}//Koniec: dzwiek_ID_Check_OK


void dzwiek_ID_Check_NotOK(){//Dźwięk wydawany podczas nieudanego odczytu karty RFID
	for(i = 0; i<300; i++){
		HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET);
		delay_us(Tony[0]);
		HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);
		delay_us(Tony[0]);}
}//Koniec: dzwiek_ID_Check_NotOK


void dzwiek_permOpenON(){//Dźwięk po otwarciu stałym furtki
	for(i = 0; i<300; i++){
		HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET);
		delay_us(Tony[6]);
		HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);
		delay_us(Tony[6]);
	}
}//Koniec: dzwiek_permOpenON


void dzwiek_doorClose(){//Dźwięk po zamknięciu furtki
	for(i = 0; i<50; i++){
		HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET);
		delay_us(Tony[2]);
		HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);
		delay_us(Tony[2]);
	}
}//Koniec: dzwiek_doorClose


void dzwiek_czyszczenieFlash(){//Dźwięk przy zresetowaniu flash
	for(uint8_t j = 0; j<=6; j++){
		for(i = 0; i<60; i++){
			HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET);
			delay_us(Tony[j]);
			HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);
			delay_us(Tony[j]);
		}
		HAL_Delay(30);
	}
	for(uint8_t h = 6; h>0; h--){
		for(i = 0; i<60; i++){
			HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET);
			delay_us(Tony[h]);
			HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);
			delay_us(Tony[h]);
		}
		HAL_Delay(30);
	}
	for(i = 0; i<60; i++){
	HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET);
	delay_us(Tony[0]);
	HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);
	delay_us(Tony[0]);
	}
}//Koniec: dzwiek_czyszczenieFlash


void dzwiek_Inicjalizacja(){//Dźwięk po udanej inicjalizacji płytki
	for(uint8_t j = 0; j <2; j++){
		for(i = 0; i<50; i++){
			HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET);
			delay_us(Tony[5]);
			HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);
			delay_us(Tony[5]);
		}
		HAL_Delay(50);
	}
	for(i = 0; i<150; i++){
		HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET);
		delay_us(Tony[6]);
		HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);
		delay_us(Tony[6]);
	}
}//Koniec: dzwiek_Inicjalizacja

void dzwiek_flash_oczekiwanie(){
	for(i = 0; i<50; i++){
		HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET);
		delay_us(Tony[6]);
		HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);
		delay_us(Tony[6]);
	}
}

void InicjalizacjaOdbiornika(){//Inicjalizacja nRF24 w trybie odbiornika
	HAL_GPIO_WritePin(NRF_VCC_GPIO_Port, NRF_VCC_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(NRF_VCC_GPIO_Port, NRF_VCC_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	NRF24_begin(NRF_CE_GPIO_Port, NRF_CSN_Pin, NRF_CE_Pin, hspi3);
	nrf24_DebugUART_Init(huart2);
	NRF24_setChannel(35);
	NRF24_setPayloadSize(32);
	NRF24_setCRCLength(RF24_CRC_16);
	NRF24_setDataRate(RF24_250KBPS);
	NRF24_openReadingPipe(0, AdresOdbiorczy);
	NRF24_startListening();
}//Koniec: InicjalizacjaOdbiornika


void resetowanieDiody(){//Resetowanie diody nadajnika, w zależności od stanu furtki
	NRF24_stopListening();
	NRF24_openWritingPipe(AdresNadawczy);
	if(Door_PermOpen)
		NRF24_write(resetDiody, 32);
	else
		NRF24_write(resetDiody2, 32);
	NRF24_startListening();
	LicznikPerm = 0;
}//Koniec: resetowanieDiody


void resetRamki(){ // Zerowanie ramki
	  for(i = 0; i <= 255; i++)
		  ramka[i] = 0;
	  poczatekRamki = 0;
	  odbiorRamki = 0;
	  koniecRamki = 0;
	  ramkaReset = 0;
	  Licznik_Ramka = 0;
	  ramkaIDX = 0;
	  przekroczonoZakres = 0;
	  bladKodowania = 0;
} //Koniec resetu ramki


void resetKomendy(){ //Zerowanie komendy
	  for(i = 0; i <= 246; i ++)
		  komenda[i] = 0;
} //Koniec resetu komendy


void flashClear(){
	uint32_t miejsceFlashEmpty[] = {0x8032010};
	uint32_t zerujFlash[4] = {0, 0, 0, 0};
	uint32_t Flash_Zerowanie_Adres = 0x8032010;
	Flash_Write_Data(0x08031810, miejsceFlashEmpty);
	for(i = 0 ; i <= 150; i++){
		Flash_Write_Data(Flash_Zerowanie_Adres, zerujFlash);
		Flash_Zerowanie_Adres = Flash_Zerowanie_Adres + 0x800;
	}
	USART_wysylka("Zresetowano flash!\n\r");
	if(dzwiekPrzelacznik)
		dzwiek_czyszczenieFlash();
}
/*-------------------------------------KONIEC FUNKCJI-------------------------------------------*/


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /*--------------------------------INICJALIZACJA-------------------------------------------*/



  HAL_TIM_Base_Start(&htim1);//Inicjalizacja licznika mikrosekundowego
  HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET); //wyłączenie pinu głośnika
  HAL_GPIO_WritePin(RELAY_OUT_GPIO_Port, RELAY_OUT_Pin, GPIO_PIN_RESET);//Upewnienie się, że furtka jest zamknięta przy inicjalizacji
  InicjalizacjaOdbiornika(); //ustawienie modułu nRF24 w trybie odbiornika
  HAL_Delay(10);
  cs_set();			 	//Ustawienie pinu SPI dla RC522
  RC522_SetReset(); 	//Ustawienie pinu Reset dla RC522
  MFRC522_Init();		//Inicjalizacja RC522 dla SPI2
  HAL_Delay(10);
  dzwiek_Inicjalizacja();



/*------------------------------KONIEC INICJALIZACJI--------------------------------------*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  HAL_UART_Receive_IT(&huart2, &USART_ODBBuf[0], 1);


  while (1)//-------------------------------WHILE-----------------------------------------------------------------------------------
  {

	  Czas_Opoznienie = Czas_OtwarciaFurtki + 300;


//----------Sekwencja wciskania przycisku dla dostępu do zapisu flash-----------

	  if(Opoznienie)//Przycisk wciśnięty
		 if(HAL_GPIO_ReadPin(PN_GPIO_Port, PN_Pin)==0){
			if(!PN_Licznik_Toggle){
				PN_Licznik_Toggle = 1;
				PN_OFF_Licznik_Zapisany = PN_OFF_Licznik;
				PN_OFF_Licznik = 0;
			}
		  }//Koniec: Przycisk wciśnięty


	  if(Opoznienie)//Przycisk niewciśnięty
	  		  if(PN_Licznik_Toggle)
	  	  		  if(HAL_GPIO_ReadPin(PN_GPIO_Port, PN_Pin)==1){
	  	  			  PN_Licznik_Toggle = 0;
	  	  			  PN_ON_Licznik_Zapisany = PN_ON_Licznik;
	  	  			  PN_ON_Licznik = 0;
	  	  		  }//Koniec: Przycisk niewciśnięty


	  if(Opoznienie)//Etap 1: Przycisk był wciśnięty od 2s do 4s
		  if(!PN_Verify1)
			  if((PN_ON_Licznik_Zapisany>Czas_ZapisPrzycisk_E1D)&&(PN_ON_Licznik_Zapisany<Czas_ZapisPrzycisk_E1G)){
				  PN_Verify1 = 1;
				  PN_OFF_Licznik_Zapisany = 0;
			  }//Koniec: Etap 1: Przycisk był wciśnięty od 2s do 4s


	  if(Opoznienie)//Etap 2: Przycisk nie był wciśnięty od 3s do 6s
		  if(PN_Verify1)
			  if((PN_OFF_Licznik_Zapisany>Czas_ZapisPrzycisk_E2D)&&(PN_OFF_Licznik_Zapisany<Czas_ZapisPrzycisk_E2G)){
				  PN_Verify2 = 1;
				  PN_Verify1 = 0;
			  }//Koniec: Etap 2: Przycisk nie był wciśnięty od 3s do 6s


	  if(Opoznienie)//Etap 3: Przycisk był wciśnięty od 5s do 10s
		  if(PN_Verify2)
			  if((PN_ON_Licznik_Zapisany>Czas_ZapisPrzycisk_E3D)&&(PN_ON_Licznik_Zapisany<Czas_ZapisPrzycisk_E3G)){
				  ID_Zapis = 1;
				  PN_Verify1 = 0;
				  PN_Verify2 = 0;
			  }//Koniec: Etap 3: Przycisk był wciśnięty od 5s do 10s

//uprawniony brelok do zeskanowania
	  if(Opoznienie)//Powyżej 20s, kasowanie pamięci flash dla RFID
		  if((PN_ON_Licznik_Zapisany>Czas_WyczyszczenieFlash)||(czyszczenieFlash)){
			  trigger_flashClear = 1;
	  	  	  if(dzwiekPrzelacznik)
	  	  		  dzwiek_flash_oczekiwanie();
	  		PN_ON_Licznik_Zapisany = 0;
	  		czyszczenieFlash = 0;
		  }//Koniec: kasowanie pamięci flash

	  if(trigger_flashClear)
	  	  if (!MFRC522_Request(PICC_REQIDL, ID))
	  	  		if (!MFRC522_Anticoll(ID))
	  	  			if(Flash_IDCheck(ID[0])){
	  	  				flashClear();
	  	  				trigger_flashClear = 0;
	  	  				licznik_flashClear = 0;
	  	  				Opoznienie = 0;
	  	  			}

	  if(PN_OFF_Licznik>Czas_ZapisPrzycisk_Zerowanie){//Powyżej tego czasu licznik i weryfikacje się zerują
		  PN_Verify1 = 0;
		  PN_Verify2 = 0;
		  PN_OFF_Licznik = 0;
	  }//Koniec: Zerowanie licznika

//---------Koniec: //Sekwencja wciskania przycisku dla dostępu do zapisu flash---------


	  if(Opoznienie)//Zapis RFID do pamięci flash
	  	  if(ID_Zapis)
	  		  if (!MFRC522_Request(PICC_REQIDL, ID))
	  			if (!MFRC522_Anticoll(ID)){
	  				  Opoznienie = 0;
	  				  Flash_IDSave(ID[0]);
	  				  if(dzwiekPrzelacznik)
	  					  dzwiek_ID_Zapis();
	  				  ID_Zapis = 0;
	  				  Licznik_ID_Zapis = 0;
	  				  USART_wysylka("Pomyslnie zapisano karte do pamieci flash!\n\r");
	  				  HAL_GPIO_WritePin(LDG_GPIO_Port, LDG_Pin, GPIO_PIN_RESET);
	  			}
	  	  	  	  //Koniec: Zapis RFID do pamięci flash


	  if(!ID_Zapis)//Odczyt i sprawdzenie RFID w celu otwarcia furtki
		  if(!Door_PermOpen)
			  if(Opoznienie)
				  if(!trigger_flashClear)
					  if (!MFRC522_Request(PICC_REQIDL, ID)){
						  if (!MFRC522_Anticoll(ID)){
							  ID_Weryfikacja = Flash_IDCheck(ID[0]);
							  if(ID_Weryfikacja){
								  Door_ChangeState();
								  if(dzwiekPrzelacznik)
									  dzwiek_ID_Check_OK();
								  Door_Open = 1;
							  }
							  else if(dzwiekPrzelacznik)
								  dzwiek_ID_Check_NotOK();
							  Opoznienie = 0;
						  }
					  }//Koniec: Odczyt i sprawdzenie RFID w celu otwarcia furtki


	  if(Opoznienie)//Odbiór klucza modułu radiowego nRF24
		  if(NRF24_available()){
			  NRF24_read(klucz, 32);
			  if(Door_PermOpen){
				  if((atoi(klucz) == kluczKrotki)||(atoi(klucz) == kluczDlugi)){//zamknięcie furtki
					  USART_wysylka("Zamkniecie furtki.\n\r");
					  Door_ChangeState();
					  Door_PermOpen = 0;
					  LDG_Toggle();
					  if(dzwiekPrzelacznik)
						  dzwiek_doorClose();
					  Opoznienie = 0;
				  }}
			  else if(atoi(klucz) == kluczKrotki)//czasowe otwarcie furtki
			  {
				  USART_wysylka("Poprawna weryfikacja klucza stanu krotkiego. Otwarcie furtki na %dms.\n\r",Czas_OtwarciaFurtki);
				  Door_ChangeState();
				  if(dzwiekPrzelacznik)
					  dzwiek_ID_Check_OK();
				  Door_Open = 1;
				  Opoznienie = 0;
			  }
			  else if(atoi(klucz) == kluczDlugi)//otwarcie furtki do momentu jej zamknięcia pilotem
			  {
				  USART_wysylka("Poprawna weryfikacja klucza stanu dlugiego. Otwarcie furtki do momentu ponownego jej zamkniecia pilotem lub komenda.\n\r");
				  Door_ChangeState();
				  Door_PermOpen = 1;
				  if(dzwiekPrzelacznik)
					  dzwiek_permOpenON();
				  LDG_Toggle();
				  Opoznienie = 0;
			  }
			  memset(klucz,' ',50);
		  }//Koniec: odbiór klucza nRF24


	  if(resetDiody_wysylka){//Resetowanie diody nadajnika
		  resetowanieDiody();
		  resetDiody_wysylka = 0;
	  }//Koniec: resetDiody_wysylka



	  //Początek obsługi komunikacji UART---------------

		if (USART_ODB_NotEmpty()) {		//Odebrano znak
			znak = USART_PobierzZnak();
			if (znak == 60) {	//znak początku ramki
					resetRamki();
					odbiorRamki = 1;
					ramkaReset = 1;
			}
			else if (odbiorRamki) {
				if (ramkaIDX > 255) { //Przekroczenie dozwolonej długości ramki
					przekroczonoZakres = 1;
					koniecRamki = 1;
				} else {
					if(kodowanie){
						if(znak == 112)		//jeśli wystąpiło \p
							ramka[ramkaIDX] = 60;	//podstaw <
						else if(znak == 107)	//jeśli wystąpiło \k
							ramka[ramkaIDX] = 62;	//podstaw >
						else if(znak == 92){	//jeśli wysłano ukośnik kodowany
							ramka[ramkaIDX] = znak;		//podstaw ukośnik
							kodowanie ++;	//i zwiększ, aby zorientować się że wystąpił ukośnik kodowany
						}
						else if(ramkaIDX > 7){ //został podany inny znak po znaku kodującym
							bladKodowania = 1; //ustaw zmienną powiadamiającą o błędzie
							koniecRamki = 1;   //i zakończ kolekcjonowanie ramki
						}
						else
							resetRamki(); //zakończ i zresetuj od razu, bo brak podanego nadawcy
						kodowanie--;
					}
					else
						ramka[ramkaIDX] = znak;
					ramkaIDX++;
					if (znak == 62 && !kodowanie) { //Odebrano znak końca ramki
						if (ramkaIDX > 7) {
							koniecRamki = 1;
							dlugoscRamki = ramkaIDX;
							ramkaIDX = 0;
							poczatekRamki = 0;
							odbiorRamki = 0;
						} else
							resetRamki(); //brak podanego nadawcy i komendy - błędna ramka
					}
					if(znak == 92 && !kodowanie){	//otrzymano ukośnik, znak kodowany
						kodowanie = 1;
						ramkaIDX--;
					}
					else if(znak == 92 && kodowanie)	//wystąpił ukośnik zakodowany
						kodowanie = 0;
				}
			}
		}
		  if(koniecRamki){
					  for(uint8_t t = 4; t <= 6; t++)
						  OdbiorcaSpr[t - 4] = ramka[t];
					  if((!strcmp(OdbiorcaSpr, Odbiorca))) //sprawdź czy ramka adresowana była do płytki (NCL)
					  {
						  if((ramka[3] == 95) && (ramka[7] == 95)){ //Sprawdzenie, czy podkreślniki są w odpowiednich miejscach
						  for(i = 0; i <= 14; i++) // Przepisanie dla komendy zmiany czasu
							  komenda[i] = ramka[i  + 8];
						  for(i = 0; i <= 2; i++) // Odebranie nadawcy z ramki
							  Nadawca[i] = ramka[i];
						  if(przekroczonoZakres)
						  {
							  if (dzwiekPrzelacznik)
									dzwiek_ID_Check_NotOK();
							  USART_wysylka("NCL_%s_Przekroczono maksymalna dlugosc ramki.\n\r", Nadawca);
						  }
						  else if(bladKodowania)
						  {
							  if(dzwiekPrzelacznik)
								  dzwiek_ID_Check_NotOK();
							  USART_wysylka("NCL_%s_Blad Kodowania - wpisano niedozwolony znak po znaku kodujacym.\n\r",Nadawca);

						  }
						  else
						  {
							  if(!strcmp(komenda, komendaZmienCzas)) // Indywidualny przypadek dla komendy
								  komendaZmienCzasTrigger = 1;
							  else
							  {
								  for(i = 0; i <= 14; i ++) //Zerowanie komendy
									  komenda[i] = 0;
								  for(i = 8; i < dlugoscRamki - 1; i++) // Odebranie komendy z ramki
									  komenda[i - 8] = ramka[i];
							  }
							  //początek sprawdzania rodzaju komendy
							  if(!strcmp(komenda,komendaOtworz)){
								  if(!(HAL_GPIO_ReadPin(RELAY_OUT_GPIO_Port, RELAY_OUT_Pin)))
								  {
									  Door_ChangeState();
									  if(dzwiekPrzelacznik)
										  dzwiek_ID_Check_OK();
									  Door_Open = 1;
									  Opoznienie = 0;
									  USART_wysylka("NCL_%s_Elektrozaczep zostal otwarty na czas: %dms.\n\r",Nadawca,Czas_OtwarciaFurtki);
								  }
								  else
									  USART_wysylka("NCL_%s_Elektrozaczep jest juz aktualnie otwarty.\n\r", Nadawca);
							  }
							  else if(!strcmp(komenda,komendaOtworzStale)){
								  if(!(HAL_GPIO_ReadPin(RELAY_OUT_GPIO_Port, RELAY_OUT_Pin)))
								  {
									  Door_ChangeState();
									  Door_PermOpen = 1;
									  if(dzwiekPrzelacznik)
										  dzwiek_permOpenON();
									  LDG_Toggle();
									  Opoznienie = 0;
									  USART_wysylka("NCL_%s_Elektrozaczep zostal otwarty do momentu jego zamkniecia pilotem lub komenda.\n\r",Nadawca);
								  }
								  else
									  USART_wysylka("NCL_%s_Elektrozaczep jest juz aktualnie otwarty.\n\r ",Nadawca);
							  }
							  else if(!strcmp(komenda,komendaZamknij)){
								  if(Door_PermOpen)
								  {
									  Door_ChangeState();
									  Door_PermOpen = 0;
									  LDG_Toggle();
									  if(dzwiekPrzelacznik)
										  dzwiek_doorClose();
									  Opoznienie = 0;
									  Licznik_Opoznienie = Czas_Opoznienie - 1000;
									  USART_wysylka("NCL_%s_Elektrozaczep zostal zamkniety.\n\r", Nadawca);
								  }
								  else if(HAL_GPIO_ReadPin(RELAY_OUT_GPIO_Port, RELAY_OUT_Pin))
								  {
									  Door_ChangeState();
									  Door_Open = 0;
									  Licznik = 0;
									  LDG_Toggle();
									  if(dzwiekPrzelacznik)
										  dzwiek_doorClose();
									  Opoznienie = 0;
									  Licznik_Opoznienie = Czas_Opoznienie - 1000;
									  USART_wysylka("NCL_%s_Elektrozaczep zostal zamkniety.\n\r",Nadawca);
								  }
								  else if(!(HAL_GPIO_ReadPin(RELAY_OUT_GPIO_Port, RELAY_OUT_Pin)))
									  USART_wysylka("NCL_%s_Elektrozaczep jest juz aktualnie zamkniety.\n\r",Nadawca);
							  }
							  else if(!strcmp(komenda,komendaFlashZapisz)){
								  if(!Opoznienie)
									  USART_wysylka("NCL_%s_Prosze poczekac az elektrozaczep zostanie zamkniety lub opoznienie zapisu minie.",Nadawca);
								  else if(trigger_flashClear)
									  USART_wysylka("NCL_%s_Trwa czyszczenie flash. Prosze przylozyc brylok w celu wyczyszczenia lub poczekac.",Nadawca);
								  else
								  {
								  ID_Zapis = 1;
								  USART_wysylka("NCL_%s_Zapis flash. Prosze przylozyc karte aby zapisac do pamieci.\n\r",Nadawca);
								  }
							  }
							  else if(!strcmp(komenda, komendaFlashWyczysc)){
								  if(!trigger_flashClear){
									  czyszczenieFlash = 1;
								  	  USART_wysylka("NCL_%s_Prosze przylozyc do anteny uprawniony brylok w celu wyczyszczenia (5s).",Nadawca);
								  }
								  else
									  USART_wysylka("NCL_%s_Trwa czyszczenie flash. Prosze przylozyc brylok w celu wyczyszczenia lub poczekac..",Nadawca);
							  }
							  else if(!strcmp(komenda,komendaDzwiek))
							  {
								  if(dzwiekPrzelacznik){
									  dzwiekPrzelacznik = 0;
									  USART_wysylka("NCL_%s_Dzwiek wylaczony.\n\r",Nadawca);
								  }
								  else
								  {
									  dzwiekPrzelacznik = 1;
									  dzwiek_ID_Check_OK();
									  USART_wysylka("NCL_%s_Dzwiek wlaczony.\n\r",Nadawca);
								  }
							  }
							  else if(komendaZmienCzasTrigger){
								  komendaZmienCzasTrigger = 0;
								  memset(zmianaCzasuTablica,0,5);
								  uint8_t dlugoscTabCzas = 5;
								  for(i = 23; i <= 27; i++){
									  if((ramka[i] == 0) || (ramka[i] == 62)){
										  dlugoscTabCzas = i - 23;
										  break;
									  }
									  if((ramka[i] > 47) && (ramka[i] < 58))
									  {
										  zmianaCzasuTablica[i - 23] = ramka[i] - 48;
									  }
									  else
									  {
										  zmianaCzasuError = 1;
										  break;
									  }
								  }
								  if(dlugoscRamki > 29)
									  zmianaCzasuError = 2;
								  if(!zmianaCzasuError)
								  {
									  tempCzas = 0;
									  for(i = 0; i < dlugoscTabCzas; i++)
										  tempCzas = 10 * tempCzas + zmianaCzasuTablica[i];
									  if((tempCzas >0) && (tempCzas <= 50000))
									  {
										  Czas_OtwarciaFurtki = tempCzas;
										  USART_wysylka("NCL_%s_Ustawiono czas otwarcia elektrozaczepu na %d ms.\n\r",Nadawca, Czas_OtwarciaFurtki);
									  }
									  else
										  USART_wysylka("NCL_%s_Podano niewlasciwy czas otwarcia elektrozaczepu. Dopuszczalny zakres to 1ms > CZAS > 50001ms.\n\r", Nadawca);
								  }
								  else if(zmianaCzasuError == 1)
									  USART_wysylka("NCL_%s_Podano niewlasciwe znaki podajac nowy czas otwarcia elektrozaczepu.\n\r ", Nadawca);
								  else
									  USART_wysylka("NCL_%s_Podano zbyt wiele znakow w polu zmiany czasu.\n\r", Nadawca);
								  zmianaCzasuError = 0;
							  }
							  else
							  {
								  USART_wysylka("NCL_%s_Wprowadzono zla komende.\n\r",Nadawca);
								  if(dzwiekPrzelacznik)
									  dzwiek_ID_Check_NotOK();
							  }
						  }
					  }
						  else{
							  USART_wysylka("NCL_%s_Nie znaleziono w odpowiednich miejscach znakow '_'.\n\r",Nadawca);
							  if(dzwiekPrzelacznik)
							  dzwiek_ID_Check_NotOK();
						  }
				  }
			  resetRamki();
			  resetKomendy();
		  }





  }//-----------------------------------KONIEC WHILE---------------------------------------------------------------------------------------




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
