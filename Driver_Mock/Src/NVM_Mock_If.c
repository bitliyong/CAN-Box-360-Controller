#include "NVM_Mock_If.h"
#include "eeprom.h"
NVM_Info_t nvmInfo;
Menu_Setting_t menuSetting;

NVM_Info_t* NVM_GetNvMInfo(void)
{
    nvmInfo.menuSetting = &menuSetting;

//     menuSetting.settings_b.Poweron_Startup_Sts = GetFullViewPowerOnSelectionEE();//1u;
//     menuSetting.settings_b.Steering_Open_Sts = GetFullViewTurnSelectionEE();//1u;
//     menuSetting.settings_b.vehicleSpeed_threshold = GetFullViewSpeedSelectionEE();//40u;
// 		menuSetting.settings_b.Radar_Obstacle_Open_Sts = GetFullViewRadarSelectionEE(); 
    return &nvmInfo;
}