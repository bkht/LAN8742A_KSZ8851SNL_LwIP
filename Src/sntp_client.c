#include <app_ethernet.h>
#include <sntp.h>
#include "sntp_client.h"
#include <time.h>

/*Codes_SRS_SNTP_LWIP_30_001: [ The ntp_lwip shall implement the methods defined in sntp.h. ]*/
// 1.nl.pool.ntp.org 83.98.155.30
#define SNTP_CONF_IPADDR0   185
#define SNTP_CONF_IPADDR1   255
#define SNTP_CONF_IPADDR2   55
#define SNTP_CONF_IPADDR3   20

void SNTP_ObtainTime(void)
{
//  return;

  dmc_puts("SNTP_ObtainTime\n");
  SNTP_Init();

  // wait for time to be set
  time_t now = 0;
  struct tm timeinfo = { 0 };
  int retry = 0;
  const int retry_count = 10;
  while(timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
//    dmc_puts("Waiting for system time to be set... (");
//    dmc_putc('.');
    HAL_Delay(20);
//    dmc_putint(retry);
//    dmc_putc('/');
//    dmc_putint(retry_count);
//    dmc_puts(")\n");

    time(&now);
    localtime_r(&now, &timeinfo);
  }
//  dmc_puts("\n");

  dmc_putint2(timeinfo.tm_hour, '0');
  dmc_putc(':');
  dmc_putint2(timeinfo.tm_min, '0');
  dmc_putc(':');
  dmc_putint2(timeinfo.tm_sec, '0');
  dmc_putcr();

  //    ESP_ERROR_CHECK( esp_wifi_stop() );
}

/*Codes_SRS_SNTP_LWIP_30_002: [ The serverName parameter shall be an NTP server URL which shall not be validated. ]*/
/*Codes_SRS_SNTP_LWIP_30_003: [ The SNTP_SetServerName shall set the NTP server to be used by ntp_lwip and return 0 to indicate success.]*/
//
// SNTP_SetServerName must be called before `SNTP_Init`.
// The character array pointed to by `serverName` parameter must persist
// between calls to `SNTP_SetServerName` and `SNTP_Deinit` because the
// char* is stored and no copy of the string is made.
//
// SNTP_SetServerName is a wrapper for the lwIP call `sntp_setservername`
// and defers parameter validation to the lwIP library.
//
// Future implementations of this adapter may allow multiple calls to
// SNTP_SetServerName in order to support multiple servers.
//
int SNTP_SetServerName(const char* serverName)
{
    // Future implementations could easily allow multiple calls to SNTP_SetServerName
    // by incrementing the index supplied to sntp_setservername
    sntp_setservername(0, (char*)serverName);
    return 0;
}

/*Codes_SRS_SNTP_LWIP_30_004: [ SNTP_Init shall initialize the SNTP client, contact the NTP server to set system time, then return 0 to indicate success (lwIP has no failure path). ]*/
int SNTP_Init(void)
{
  static ntp_initialized = 0;
  static struct ip4_addr sntpserver_ip_addr;


//  dmc_puts("sntp_setserver\n");
  if (!ntp_initialized)
  {
    dmc_puts("Initializing SNTP\n");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    //  // 1.nl.pool.ntp.org 83.98.155.30
//    IP4_ADDR(&sntpserver_ip_addr, SNTP_CONF_IPADDR0, SNTP_CONF_IPADDR1, SNTP_CONF_IPADDR2, SNTP_CONF_IPADDR3);
//    sntp_setserver(0, &sntpserver_ip_addr);
    sntp_setservername(0, (char*)"nl.pool.ntp.org");
//    sntp_setservername(1, (char*)"time.windows.com");
//    sntp_setservername(2, (char*)"time.nist.gov");
//    sntp_set_timezone(0);

    sntp_init();
//    time_t ts = 0;
//    // Before 1980 is uninitialized
//    while (ts < 10 * 365 * 24 * 3600)
//    {
//        HAL_Delay(1000);
//        ts = get_time(NULL);
//
//    }
    long gmtOffset_sec = 0;
    int daylightOffset_sec = 0;
    setTimeZone(-gmtOffset_sec, daylightOffset_sec);
    dmc_puts("SNTP initialization complete\n");
  }

  ntp_initialized = 1;
  return 0;
}

/*Codes_SRS_SNTP_LWIP_30_005: [ SNTP_Denit shall deinitialize the SNTP client. ]*/
void SNTP_Deinit(void)
{
    sntp_stop();
}

void setTimeZone(long offset, int daylight)
{
    char cst[17] = {0};
    char cdt[17] = "DST";
    char tz[33] = {0};

    if(offset % 3600){
        sprintf(cst, "UTC%ld:%02u:%02u", offset / 3600, abs((offset % 3600) / 60), abs(offset % 60));
    } else {
        sprintf(cst, "UTC%ld", offset / 3600);
    }
    if(daylight != 3600){
        long tz_dst = offset - daylight;
        if(tz_dst % 3600){
            sprintf(cdt, "DST%ld:%02u:%02u", tz_dst / 3600, abs((tz_dst % 3600) / 60), abs(tz_dst % 60));
        } else {
            sprintf(cdt, "DST%ld", tz_dst / 3600);
        }
    }
    sprintf(tz, "%s%s", cst, cdt);
    setenv("TZ", tz, 1);
    tzset();
}


