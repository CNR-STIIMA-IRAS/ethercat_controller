#ifndef ETHERCAT_CONTROLLERS__TRANSITION_UTILITIES_HPP_
#define ETHERCAT_CONTROLLERS__TRANSITION_UTILITIES_HPP_
#include <memory>
#include <string>
#include <vector>


namespace ethercat_controllers
{

enum CommandID  { CMD_SHUTDOWN             = 0
                , CMD_SWITCH_ON
                , CMD_DISABLE_VOLTAGE
                , CMD_QUICK_STOP
                , CMD_DISABLE
                , CMD_ENABLE
                , CMD_FAULT_RESET
                , CMD_COMPLETE_RESET
                , CMD_INTERNAL
                , CMD_LOOPBACK
                };

enum TransitionID   { TRANSITION_LOOPBACK   = 0
                    , TRANSITION_0_INTERNAL 
                    , TRANSITION_1_INTERNAL
                    , TRANSITION_2
                    , TRANSITION_3
                    , TRANSITION_4
                    , TRANSITION_5
                    , TRANSITION_6
                    , TRANSITION_7
                    , TRANSITION_8
                    , TRANSITION_9
                    , TRANSITION_10
                    , TRANSITION_11
                    , TRANSITION_12
                    , TRANSITION_13_INTERNAL
                    , TRANSITION_14_INTERNAL
                    , TRANSITION_15
                    , TRANSITION_16
                    };
enum TransitionType { TRANSITION_INTERNAL = 0
                    , TRANSITION_FWD = 1
                    , TRANSITION_BCK = 2
                    , TRANSITION_ALARM = 3 
                    , TRANSITION_FOO = 4
                    };


typedef std::map< CommandID, std::map<std::string,uint16_t> > CommandMask; 
static const  CommandMask COMMAND_MASK = { 
//bit                 7               3               2           1           0
//                FaultReset | EnableOperation | QuickStop | EnableVoltage | SwitchOn
//Shutdown            0               X               1           1           0   -- 2, 6, 8
//Switch ON           0               0               1           1           1   -- 3*
//Switch ON           0               1               1           1           1   -- 3**
//Disable Voltage     0               X               X           0           X   -- 7, 9, 10, 12
//Quick Stop          0               X               0           1           X   -- 7, 10, 11
//Disable Operation   0               0               1           1           1   -- 5
//Enable Operation    0               1               1           1           1   -- 4, 16
//Fault Reset         TR              X               X           X           X   -- 15
    {CMD_SHUTDOWN       , { {"MASK", 0b0000000010000111 },{"VAL", 0b0000000000000110}  } } //uint8_t cmd[2], cmd[0] = 0x87, cmd[1] = 0x00
  , {CMD_SWITCH_ON      , { {"MASK", 0b0000000010001111 },{"VAL", 0b0000000000000111}  } } //uint8_t cmd[2], cmd[0] = 0x8f, cmd[1] = 0x00
  , {CMD_DISABLE_VOLTAGE, { {"MASK", 0b0000000010000010 },{"VAL", 0b0000000000000000}  } } //uint8_t cmd[2], cmd[0] = 0x86, cmd[1] = 0x00
  , {CMD_QUICK_STOP     , { {"MASK", 0b0000000010000110 },{"VAL", 0b0000000000000010}  } } //uint8_t cmd[2], cmd[0] = 0x80, cmd[1] = 0x00
  , {CMD_DISABLE        , { {"MASK", 0b0000000010001111 },{"VAL", 0b0000000000000111}  } } //uint8_t cmd[2], cmd[0] = 0x8f, cmd[1] = 0x00
  , {CMD_ENABLE         , { {"MASK", 0b0000000010001111 },{"VAL", 0b0000000000001111}  } } //uint8_t cmd[2], cmd[0] = 0x8f, cmd[1] = 0x00
  , {CMD_FAULT_RESET    , { {"MASK", 0b0000000011111111 },{"VAL", 0b0000000010000000}  } } //uint8_t cmd[2], cmd[0] = 0xff, cmd[1] = 0x00
  , {CMD_COMPLETE_RESET , { {"MASK", 0b1111111111111111 },{"VAL", 0b0000000000000000}  } } // it does reset the control word!
  , {CMD_INTERNAL       , { {"MASK", 0b0000000000000000 },{"VAL", 0b0000000000000000}  } } // it does not change the control word!
  , {CMD_LOOPBACK       , { {"MASK", 0b0000000000000000 },{"VAL", 0b0000000000000000}  } } };


typedef std::map< CommandID, std::vector< TransitionID > >  CommandTansitionFeasibilityMap; // one-to-one
static const  CommandTansitionFeasibilityMap COMMAND_TRANSITION_MAP =  {
//bit                 7               3               2           1           0
//                FaultReset | EnableOperation | QuickStop | EnableVoltage | SwitchOn
//Shutdown            0               X               1           1           0   -- 2, 6, 8
//Switch ON           0               0               1           1           1   -- 3*
//Switch ON           0               1               1           1           1   -- 3**
//Disable Voltage     0               X               X           0           X   -- 7, 9, 10, 12
//Quick Stop          0               X               0           1           X   -- 7, 10, 11
//Disable Operation   0               0               1           1           1   -- 5
//Enable Operation    0               1               1           1           1   -- 4, 16
//Fault Reset         TR              X               X           X           X   -- 15
      {CMD_SHUTDOWN       , {TRANSITION_2,TRANSITION_6,TRANSITION_8}}
    , {CMD_SWITCH_ON      , {TRANSITION_3 }}
    , {CMD_DISABLE_VOLTAGE, {TRANSITION_7,TRANSITION_9,TRANSITION_10,TRANSITION_12 }}
    , {CMD_QUICK_STOP     , {TRANSITION_7,TRANSITION_10,TRANSITION_11 }}
    , {CMD_DISABLE        , {TRANSITION_5 }}
    , {CMD_ENABLE         , {TRANSITION_4,TRANSITION_16 }}
    , {CMD_FAULT_RESET    , {TRANSITION_15 }}
    , {CMD_COMPLETE_RESET , {TRANSITION_0_INTERNAL }}
    , {CMD_INTERNAL       , {TRANSITION_1_INTERNAL, TRANSITION_13_INTERNAL }}
    , {CMD_LOOPBACK       , {TRANSITION_LOOPBACK}}
    };
typedef std::map<TransitionID, TransitionType> TransitionTypeMap;
static const  TransitionTypeMap TRANSITION_TYPE_MAP =  {
   { TRANSITION_LOOPBACK      , TRANSITION_FOO      }
  ,{ TRANSITION_0_INTERNAL    , TRANSITION_INTERNAL }
  ,{ TRANSITION_1_INTERNAL    , TRANSITION_INTERNAL }
  ,{ TRANSITION_2             , TRANSITION_FWD      }
  ,{ TRANSITION_3             , TRANSITION_FWD      }
  ,{ TRANSITION_4             , TRANSITION_FWD      }
  ,{ TRANSITION_5             , TRANSITION_BCK      }
  ,{ TRANSITION_6             , TRANSITION_BCK      }
  ,{ TRANSITION_7             , TRANSITION_BCK      }
  ,{ TRANSITION_8             , TRANSITION_BCK      }
  ,{ TRANSITION_9             , TRANSITION_BCK      }
  ,{ TRANSITION_10            , TRANSITION_ALARM    }
  ,{ TRANSITION_11            , TRANSITION_ALARM    }
  ,{ TRANSITION_12            , TRANSITION_ALARM    }
  ,{ TRANSITION_13_INTERNAL   , TRANSITION_INTERNAL }
  ,{ TRANSITION_14_INTERNAL   , TRANSITION_INTERNAL }
  ,{ TRANSITION_15            , TRANSITION_ALARM    }
  ,{ TRANSITION_16            , TRANSITION_FWD      } };
            



typedef std::vector< std::pair< CommandID, uint8_t > >    FeasibleStateCommands;     // one-to-many

typedef std::vector< std::pair< TransitionID, uint8_t > > FeasibleStateTransitions;     

CommandID to_commandid( const TransitionID& transition )
{
  CommandTansitionFeasibilityMap::const_iterator it;
  for( it = COMMAND_TRANSITION_MAP.begin(); it != COMMAND_TRANSITION_MAP.end(); it++ )
  {
    std::vector< TransitionID >::const_iterator jt = std::find (it->second.begin(), it->second.end(), transition);
    if (jt != it->second.end() )
      return it->first;
  }
  throw std::runtime_error("Command is not valid..wierd error");
}


typedef std::map< uint8_t, FeasibleStateTransitions >     StateTransitionFeasibilityMap;
static const StateTransitionFeasibilityMap STATE_TRANSITIONS_MAP = { 
    {ethercat_msgs::msg::Cia402DriveStates::STATE_START                , { std::make_pair(TRANSITION_LOOPBACK   , ethercat_msgs::msg::Cia402DriveStates::STATE_START              )   
                                                                        ,   std::make_pair(TRANSITION_0_INTERNAL , ethercat_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED    ) } }
  , {ethercat_msgs::msg::Cia402DriveStates::STATE_NOT_READY_TO_SWITCH_ON  , { std::make_pair(TRANSITION_LOOPBACK   , ethercat_msgs::msg::Cia402DriveStates::STATE_NOT_READY_TO_SWITCH_ON)
                                                                        ,   std::make_pair(TRANSITION_1_INTERNAL , ethercat_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED    ) } }
  , {ethercat_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED      , { std::make_pair(TRANSITION_LOOPBACK   , ethercat_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED    )
                                                                        ,   std::make_pair(TRANSITION_2          , ethercat_msgs::msg::Cia402DriveStates::STATE_READY_TO_SWITCH_ON    ) } }
  , {ethercat_msgs::msg::Cia402DriveStates::STATE_READY_TO_SWITCH_ON      , { std::make_pair(TRANSITION_LOOPBACK   , ethercat_msgs::msg::Cia402DriveStates::STATE_READY_TO_SWITCH_ON    )
                                                                        ,   std::make_pair(TRANSITION_3          , ethercat_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON           )
                                                                        ,   std::make_pair(TRANSITION_7          , ethercat_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED           ) } }
  , {ethercat_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON             , { std::make_pair(TRANSITION_LOOPBACK   , ethercat_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON           )
                                                                        ,   std::make_pair(TRANSITION_4          , ethercat_msgs::msg::Cia402DriveStates::STATE_OPERATION_ENABLED     ) 
                                                                        ,   std::make_pair(TRANSITION_6          , ethercat_msgs::msg::Cia402DriveStates::STATE_READY_TO_SWITCH_ON    ) 
                                                                        ,   std::make_pair(TRANSITION_10         , ethercat_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED    ) } }
  , {ethercat_msgs::msg::Cia402DriveStates::STATE_OPERATION_ENABLED       , { std::make_pair(TRANSITION_LOOPBACK   , ethercat_msgs::msg::Cia402DriveStates::STATE_OPERATION_ENABLED     )
                                                                        ,   std::make_pair(TRANSITION_5          , ethercat_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON           ) 
                                                                        ,   std::make_pair(TRANSITION_8          , ethercat_msgs::msg::Cia402DriveStates::STATE_READY_TO_SWITCH_ON    ) 
                                                                        ,   std::make_pair(TRANSITION_9          , ethercat_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED    ) 
                                                                        ,   std::make_pair(TRANSITION_11         , ethercat_msgs::msg::Cia402DriveStates::STATE_QUICK_STOP_ACTIVE     ) } }
  , {ethercat_msgs::msg::Cia402DriveStates::STATE_QUICK_STOP_ACTIVE       , { std::make_pair(TRANSITION_LOOPBACK   , ethercat_msgs::msg::Cia402DriveStates::STATE_QUICK_STOP_ACTIVE     )
                                                                        ,   std::make_pair(TRANSITION_12         , ethercat_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED    )
                                                                        ,   std::make_pair(TRANSITION_16         , ethercat_msgs::msg::Cia402DriveStates::STATE_OPERATION_ENABLED     ) } }
  , {ethercat_msgs::msg::Cia402DriveStates::STATE_FAULT_REACTION_ACTIVE   , { std::make_pair(TRANSITION_LOOPBACK   , ethercat_msgs::msg::Cia402DriveStates::STATE_FAULT_REACTION_ACTIVE )
                                                                        ,   std::make_pair(TRANSITION_14_INTERNAL, ethercat_msgs::msg::Cia402DriveStates::STATE_NOT_READY_TO_SWITCH_ON) } }
  , {ethercat_msgs::msg::Cia402DriveStates::STATE_FAULT                   , { std::make_pair(TRANSITION_LOOPBACK   , ethercat_msgs::msg::Cia402DriveStates::STATE_FAULT                 )
                                                                        ,   std::make_pair(TRANSITION_15         , ethercat_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED    ) } }
  };



typedef std::map< TransitionID, std::string >  TransitionIDStrings;
static const TransitionIDStrings TRANSITIONID_STRINGS =
    { {TRANSITION_LOOPBACK      , "TRANSITION_LOOPBACK"  }
    , {TRANSITION_0_INTERNAL    , "TRANSITION_0_INTERNAL"}
    , {TRANSITION_1_INTERNAL    , "TRANSITION_1_INTERNAL"}
    , {TRANSITION_2             , "TRANSITION_2"}
    , {TRANSITION_3             , "TRANSITION_3"}
    , {TRANSITION_4             , "TRANSITION_4"}
    , {TRANSITION_5             , "TRANSITION_5"}
    , {TRANSITION_6             , "TRANSITION_6"}
    , {TRANSITION_7             , "TRANSITION_7"}
    , {TRANSITION_8             , "TRANSITION_8"}
    , {TRANSITION_9             , "TRANSITION_9"}
    , {TRANSITION_10            , "TRANSITION_10"}
    , {TRANSITION_11            , "TRANSITION_11"}
    , {TRANSITION_12            , "TRANSITION_12"}
    , {TRANSITION_13_INTERNAL   , "TRANSITION_13_INTERNAL"}
    , {TRANSITION_14_INTERNAL   , "TRANSITION_14_INTERNAL"}
    , {TRANSITION_15            , "TRANSITION_15"}
    , {TRANSITION_16            , "TRANSITION_16"} };
}  // namespace ethercat_controllers
#endif