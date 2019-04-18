#include <app_ethernet.h>
#include <sntp.h>
#include "sntp_client.h"
#include <time.h>

#define SNTP_CLIENT_DEBUG     0

void SNTP_ObtainTime(void)
{
  SNTP_Init();

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
//  static struct ip4_addr sntpserver_ip_addr;
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};


//  dmc_puts("sntp_setserver\n");
  if (!ntp_initialized)
  {
#if (SNTP_CLIENT_DEBUG)
    dmc_puts("Initializing SNTP\n");
#endif
//    sntp_setoperatingmode(SNTP_OPMODE_POLL);
//    sntp_setservername(0, (char*)"0.nl.pool.ntp.org");
//    sntp_set_timezone(0);
    sntp_init();

    int retry = 0;
    const int retry_count = 10;

    do
    {
      HAL_Delay(20);
      DMC_I2cRtcGetDateAndTime(&sTime, &sDate);
    }
    while(sDate.Year < 19 && ++retry < retry_count);

#if (SNTP_CLIENT_DEBUG)
    dmc_puts("SNTP initialization complete\n");
#endif
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


