// 该文件是裁判系统对具体的数据进行定义与封装
#ifndef REFEREEDATA_H
#define REFEREEDATA_H

#include "main.h"
#include "protocol.h"

/**
 * @brief  0x0001
 * 比赛状态数据@3HZ
 */
typedef __packed struct {
  uint8_t game_type : 4;       // 比赛类型
  uint8_t game_progress : 4;   // 比赛阶段
  uint16_t stage_remain_time;  // 当前阶段剩余时间(s)
  uint64_t SyncTimeStamp;  // UNIX 时间，当机器人正确连接到裁判系统的 NTP 服务器后生效
} game_state_t;

// 比赛类型代号
typedef enum {
  TYPE_7V7 = 1,     // 超级对抗赛
  TYPE_SINGLE = 2,  // 单项挑战赛
  TYPE_ICRA = 3,    // 人工智能挑战赛
  TYPE_3V3 = 4,     // 联盟赛3V3
  TYPE_1V1 = 5,     // 联盟赛1V1
} game_type_t;

// 比赛阶段代号
typedef enum {
  PROGRESS_UNSTART = 0,      // 未开始
  PROGRESS_PREPARE = 1,      // 准备阶段
  PROGRESS_SELFCHECK = 2,    // 自捡阶段
  PROGRESS_5sCOUNTDOWN = 3,  // 5秒倒计时
  PROGRESS_BATTLE = 4,       // 比赛中
  PROGRESS_CALCULATING = 5,  // 比赛结算中
} game_progress_t;

/**
 * @brief  0x0002
 * 比赛结果数据
 */
typedef __packed struct  // 0002
{
  uint8_t winner;
} game_result_t;

// 比赛结果代号
typedef enum {
  WINNER_NULL = 0,  // 平局
  WINNER_RED = 1,   // 红方胜利
  WINNER_BLUE = 2,  // 蓝方胜利
} game_winner_t;

/**
 * @brief  0x0003
 * 机器人血量数据@3HZ
 */
typedef __packed struct {
  uint16_t red_1_robot_HP;
  uint16_t red_2_robot_HP;
  uint16_t red_3_robot_HP;
  uint16_t red_4_robot_HP;
  uint16_t red_5_robot_HP;
  uint16_t red_7_robot_HP;
  uint16_t red_outpost_HP;
  uint16_t red_base_HP;
  uint16_t blue_1_robot_HP;
  uint16_t blue_2_robot_HP;
  uint16_t blue_3_robot_HP;
  uint16_t blue_4_robot_HP;
  uint16_t blue_5_robot_HP;
  uint16_t blue_7_robot_HP;
  uint16_t blue_outpost_HP;
  uint16_t blue_base_HP;
} game_robot_HP_t;

/**
 * @brief  0x0101
 * 场地事件数据@3HZ
 */
typedef __packed struct {
  uint8_t HP_FRONT : 1;  // 己方补给站前补血点的占领状态，1 为已占领
  uint8_t HP_LEFT : 1;  // 己方补给站左侧（面向补给站）补血点的占领状态，1 为已占领
  uint8_t HP_RIGHT : 1;  // 己方补给站右侧（面向补给站）补血点的占领状态，1 为已占领

  uint8_t ENERGY_PLACE : 1;     // 己方能量机关激活点的占领状态，1 为已占领
  uint8_t ENERGY_SMALL_EN : 1;  // 己方小能量机关的激活状态，1 为已激活
  uint8_t ENERGY_LARGE_EN : 1;  // 己方大能量机关的激活状态，1 为已激活

  uint8_t HILL_CIRCLE_N2 : 1;  // 己方 2 号环形高地的占领状态，1 为已占领
  uint8_t HILL_N3 : 1;         // 己方 3 号梯形高地的占领状态，1 为已占领
  uint8_t HILL_N4 : 1;         // 己方 4 号梯形高地的占领状态，1 为已占领

  uint8_t BASE_VIRTUAL_HP;      // 己方基地虚拟护盾的值（0-250）
  uint16_t OUTPOST_HP : 11;     // 己方前哨站的血量（0-1500）
  uint8_t ROBOT_7_INPLACE : 1;  // 哨兵此时是否在己方巡逻区内

  uint8_t UNKNOWN : 3;  // 预留
} event_data_t;

/**
 * @brief  0x0102
 * 补给站动作标识数据，补给站弹丸释放时触发发送
 */
typedef __packed struct {
  uint8_t supply_projectile_id;    // 己方补给口ID
  uint8_t supply_robot_id;         // 补弹机器人ID
  uint8_t supply_projectile_step;  // 出弹口开闭状态
  uint8_t supply_projectile_num;   // 补弹数量
} supply_projectile_action_t;

// 补弹口代号
typedef enum {
  SUPPLY_LEFT = 1,   // 己方左侧（面向补给站）补给站口
  SUPPLY_RIGHT = 2,  // 己方右侧（面向补给站）补给站口
} supply_projectile_id_t;

// 补弹机器人代号
typedef enum {
  NO_SUPPLY_ROBOT = 0,     // 当前无机器人补弹
  RED_HERO_SUPPLY = 1,     // 红方英雄机器人补弹
  RED_N3_SUPPLY = 3,       // 红方步兵机器人补弹
  RED_N4_SUPPLY = 4,       // 红方步兵机器人补弹
  RED_N5_SUPPLY = 5,       // 红方步兵机器人补弹
  BLUE_HERO_SUPPLY = 101,  // 蓝方英雄机器人补弹
  BLUE_N3_SUPPLY = 103,    // 蓝方步兵机器人补弹
  BLUE_N4_SUPPLY = 104,    // 蓝方步兵机器人补弹
  BLUE_N5_SUPPLY = 105,    // 蓝方步兵机器人补弹
} supply_robot_id_t;

// 补弹口状态代号
typedef enum {
  SUPPLY_OFF = 0,        // 关闭
  SUPPLY_PREPARING = 1,  // 弹丸准备中
  SUPPLY_READY = 2,      // 弹丸释放
} supply_state_t;

/**
 * @brief  0x0104
 * 裁判警告数据，己方判罚/判负时触发发送
 */
typedef __packed struct {
  uint8_t level;               // 判罚等级
  uint8_t offending_robot_id;  // 违规机器人 ID
} referee_warning_t;

// 判罚等级代号
typedef enum {
  YELLOW_WARINING = 1,  // 黄牌
  RED_WARINING = 2,     // 红牌
  FAIL_WARINING = 3,    // 判负
} warning_level_t;

// 违规机器人代号
typedef enum {
  NO_WARNED_ROBOT = 0,     // 当前无机器人判罚
  RED_HERO_WARNED = 1,     // 红方英雄机器人判罚
  RED_N3_WARNED = 3,       // 红方步兵机器人判罚
  RED_N4_WARNED = 4,       // 红方步兵机器人判罚
  RED_N5_WARNED = 5,       // 红方步兵机器人判罚
  BLUE_HERO_WARNED = 101,  // 蓝方英雄机器人判罚
  BLUE_N3_WARNED = 103,    // 蓝方步兵机器人判罚
  BLUE_N4_WARNED = 104,    // 蓝方步兵机器人判罚
  BLUE_N5_WARNED = 105,    // 蓝方步兵机器人判罚
} robot_warning_id_t;

/**
 * @brief  0x0105
 * 飞镖发射时间数据，@3Hz
 */
typedef __packed struct {
  uint8_t dart_remaining_time;  // 己方飞镖发射剩余时间，单位：秒
} dart_remaining_time_t;

/**
 * @brief  0x0201
 * 机器人性能体系数据，@10HZ
 */
typedef __packed struct {
  uint8_t robot_id;                         // 机器人ID
  uint8_t robot_level;                      // 机器人等级
  uint16_t remain_HP;                       // 机器人当前血量
  uint16_t max_HP;                          // 机器人上限血量
  uint16_t shooter_id1_17mm_cooling_rate;   // 机器人 1 号 17mm 枪口热量每秒冷却值
  uint16_t shooter_id1_17mm_cooling_limit;  // 机器人 1 号 17mm 枪口热量上限
  uint16_t shooter_id1_17mm_speed_limit;    // 机器人 1 号 17mm
  // 发射机构射击初速度上限（单位：m/s）
  uint16_t shooter_id2_17mm_cooling_rate;   // 机器人 2 号 17mm 枪口热量每秒冷却值
  uint16_t shooter_id2_17mm_cooling_limit;  // 枪口热量上限
  uint16_t shooter_id2_17mm_speed_limit;    // 枪口射速上限

  uint16_t shooter_id1_42mm_cooling_rate;   // 枪口冷却速率
  uint16_t shooter_id1_42mm_cooling_limit;  // 枪口热量上限
  uint16_t shooter_id1_42mm_speed_limit;    // 枪口射速上限
  uint16_t chassis_power_limit;             // 底盘功率上限
  uint8_t mains_power_gimbal_output : 1;    // gimbal 口输出：0 为无输出，1 为 24V 输出
  uint8_t mains_power_chassis_output : 1;   // chassis 口输出：0 为无输出，1 为
                                            // 24V 输出
  uint8_t mains_power_shooter_output : 1;   // shooter 口输出：0 为无输出，1 为
                                            // 24V 输出
} robot_state_t;

/**
 * @brief  0x0202
 * 实时功率热量数据，@50Hz
 */
typedef __packed struct {
  uint16_t chassis_voltage;  // 电源管理模块 chassis 口输出电压（单位：mV）
  uint16_t chassis_current;  // 电源管理模块 chassis 口输出电流（单位：mA）
  float chassis_power;       // 底盘功率（单位：W）
  uint16_t buffer_energy;    // 缓冲能量（单位：J）
  uint16_t shooter_17mm_1_barrel_heat;  // 第 1 个 17mm 发射机构的枪口热量
  uint16_t shooter_17mm_2_barrel_heat;  // 第 2 个 17mm 发射机构的枪口热量
  uint16_t shooter_42mm_barrel_heat;    // 42mm 发射机构的枪口热量
} power_heat_data_t;

/**
 * @brief  0x0203
 * 机器人位置数据，@10Hz
 */
typedef __packed struct {
  float x;    // 本机器人位置 x 坐标，单位：m
  float y;    // 本机器人位置 y 坐标，单位：m
  float z;    // 本机器人位置 z 坐标，单位：m
  float yaw;  // 本机器人测速模块朝向，单位：度。正北为 0 度
} game_robot_pos_t;

/**
 * @brief  0x0204
 * 机器人增益数据，@3Hz
 */
typedef __packed struct {
  uint8_t recovery_buff;  // 机器人回血增益（百分比，值为 10 意为每秒回复
  // 10%最大血量）
  uint8_t cooling_buff;  // 机器人枪口冷却倍率（直接值，值为 5 意味着 5 倍冷却）
  uint8_t defence_buff;  // 机器人防御增益（百分比，值为 50 意为 50%防御增益）
  uint16_t attack_buff;  // 机器人攻击增益（百分比，值为 50 意为 50%攻击增益）
} robot_buff_t;

/**
 * @brief  0x0205
 * 空中支援时间数据，@10Hz
 */
typedef __packed struct {
  uint8_t airforce_status;  // 空中机器人状态（0 为正在冷却，1 为冷却完毕，2
                            // 为空中支援期间）
  uint8_t time_remain;  // 此状态的剩余时间（单位为 s，向下取整，即冷却时间剩余
  // 1.9s 时，此值为 1）若冷却时间为 0
  // 但未呼叫空中支援，则该值为 0
} air_support_data_t;

/**
 * @brief  0x0206
 * 伤害状态数据，伤害发生后发送
 */
typedef __packed struct {
  uint8_t armor_id : 4;  // 当扣血原因为装甲模块或测速模块时，该 4bit
                         // 组成的数值为装甲模块或测速模块的 ID
                         // 编号；其他原因扣血时，该数值为 0
  uint8_t HP_deduction_reason : 4;  // 血量变化类型
} hurt_data_t;

// 扣血原因代号
typedef enum {
  AMMO_HIT = 0,            // 装甲被弹丸攻击扣血
  SYSTEM_OFFLINE = 1,      // 裁判系统重要模块离线扣血
  SHOOT_OVER_SPEED = 2,    // 射击初速度超限扣血
  SHOOT_OVER_HEAT = 3,     // 枪口热量超限扣血
  CHASSIS_OVER_POWER = 4,  // 底盘功率超限扣血
  CRASH_HIT = 5,           // 装甲模块受到撞击扣血
} hurt_reason_t;

/**
 * @brief  0x0207
 * 实时射击数据，弹丸发射后发送
 */
typedef __packed struct {
  uint8_t bullet_type;     // 弹丸类型：1:17mm 2:42mm
  uint8_t shooter_number;  // 发射机构
  // ID,1:第一个17mm发射机构，2:第二个17mm发射机构，3:42mm发射机构
  uint8_t launching_frequency;  // 弹丸射频（单位：Hz）
  float initial_speed;          // 弹丸初速度（单位：m/s）
} shoot_data_t;

/**
 * @brief  0x0208
 * 允许发弹量，@10Hz
 */
typedef __packed struct {
  uint16_t projectile_allowance_17mm;  // 17mm 弹丸允许发弹量
  uint16_t projectile_allowance_42mm;  // 42mm 弹丸允许发弹量
  uint16_t remaining_gold_coin;        // 剩余金币数量
} projectile_allowance_t;

/**
 * @brief  0x0209
 * 机器人 RFID 状态，@3Hz
 */
typedef __packed struct {
  uint8_t self_base_buff_area : 1;          // 己方基地增益点
  uint8_t self_circle_hill_buff_area : 1;   // 己方环形高地增益点
  uint8_t enemy_circle_hill_buff_area : 1;  // 对方环形高地增益点
  uint8_t self_R3_B3_buff_area : 1;         // 己方 R3/B3 梯形高地增益点
  uint8_t enemy_R3_B3_buff_area : 1;        // 对方 R3/B3 梯形高地增益点
  uint8_t self_R4_B4_buff_area : 1;         // 己方 R4/B4 梯形高地增益点
  uint8_t enemy_R4_B4_buff_area : 1;        // 对方 R4/B4 梯形高地增益点
  uint8_t self_energy_en_area : 1;          // 己方能量机关激活点
  uint8_t self_fly_buff_before_area : 1;   // 己方飞坡增益点（靠近己方一侧飞坡前）
  uint8_t self_fly_buff_after_area : 1;    // 己方飞坡增益点（靠近己方一侧飞坡后）
  uint8_t enemy_fly_buff_before_area : 1;  // 对方飞坡增益点（靠近己方一侧飞坡前）
  uint8_t enemy_fly_buff_after_area : 1;   // 对方飞坡增益点（靠近己方一侧飞坡后）
  uint8_t self_outpost_buff_area : 1;      // 己方前哨站增益点
  uint8_t self_medical_buff_area : 1;      // 己方补血点（检测到任一均视为激活）
  uint8_t self_robot_N7_buff_area : 1;     // 己方哨兵巡逻区
  uint8_t enemy_robot_N7_buff_area : 1;    // 对方哨兵巡逻区
  uint8_t self_resources_buff_area : 1;    // 己方大资源岛增益点
  uint8_t enemy_resources_buff_area : 1;   // 对方大资源岛增益点
  uint8_t self_control_area : 1;           // 己方控制区
  uint8_t enemy_control_area : 1;          // 对方控制区
  uint16_t unknown : 12;                   // 己方控制区
} rfid_status_t;

/**
 * @brief  0x020A
 * 飞镖选手端指令数据，飞镖闸门上线后@10Hz
 */
typedef __packed struct {
  uint8_t dart_launch_opening_status;  // 当前飞镖发射站的状态
  // 1:关闭，2：正在开启或关闭中，3：开启
  uint8_t dart_attack_target;  // 飞镖的打击目标：（默认为前哨站） 0:前哨站，1:基地
  uint16_t target_change_time;  // 切换打击目标时的比赛剩余时间，单位：s，无未切换动作默认为
  // 0。
  uint16_t latest_launch_cmd_time;  // 最后一次操作手确定发射指令时的比赛剩余时间，单位：s，初始值为
  // 0。
} dart_client_cmd_t;

/**
 * @brief  0x020B
 * 地面机器人位置数据，@1Hz
 */
typedef __packed struct {
  float hero_x;        // 己方英雄机器人位置 x 轴坐标，单位：m
  float hero_y;        // 己方英雄机器人位置 y 轴坐标，单位：m
  float engineer_x;    // 己方工程机器人位置 x 轴坐标，单位：m
  float engineer_y;    // 己方工程机器人位置 y 轴坐标，单位：m
  float standard_3_x;  // 己方 3 号步兵机器人位置 x 轴坐标，单位：m
  float standard_3_y;  // 己方 3 号步兵机器人位置 y 轴坐标，单位：m
  float standard_4_x;  // 己方 4 号步兵机器人位置 x 轴坐标，单位：m
  float standard_4_y;  // 己方 4 号步兵机器人位置 y 轴坐标，单位：m
  float standard_5_x;  // 己方 5 号步兵机器人位置 x 轴坐标，单位：m
  float standard_5_y;  // 己方 5 号步兵机器人位置 y 轴坐标，单位：m
} ground_robot_position_t;

/**
 * @brief  0x020C
 * 雷达标记进度数据，@1Hz
 */
typedef __packed struct {
  uint8_t mark_hero_progress;        // 对方英雄机器人被标记进度：0—120
  uint8_t mark_engineer_progress;    // 对方工程机器人被标记进度：0—120
  uint8_t mark_standard_3_progress;  // 对方 3 号步兵机器人被标记进度：0—120
  uint8_t mark_standard_4_progress;  // 对方 4 号步兵机器人被标记进度：0—120
  uint8_t mark_standard_5_progress;  // 对方 5 号步兵机器人被标记进度：0—120
  uint8_t mark_sentry_progress;      // 对方哨兵机器人被标记进度：0—120
} radar_mark_data_t;

/**
 * @brief  0x0301
 * 机器人交互数据，发送方触发发送，频率上限为 10Hz
 */
typedef __packed struct {
  uint16_t data_cmd_id;  // 子内容 ID ,需为开放的子内容 ID
  uint16_t sender_id;    // 发送者 ID ,需与自身 ID 匹配，ID 编号详见附录
  uint16_t receiver_id;  // 接收者
  // ID,仅限己方通信,需为规则允许的车间通信接收者,若接收者为选手端，则仅可发送至发送者对应的选手端,ID
  // 编号详见附录
  uint8_t user_data[20];  // 内容数据段,数组长度最大为 113
} robot_interaction_data_t;

/**
 * @brief  0x0304
 * 键鼠遥控数据，@30Hz
 */
typedef __packed struct {
  int16_t mouse_x;           // 鼠标 x 轴移动速度，负值标识向左移动
  int16_t mouse_y;           // 鼠标 y 轴移动速度，负值标识向下移动
  int16_t mouse_z;           // 鼠标滚轮移动速度，负值标识向后滚动
  int8_t left_button_down;   // 鼠标左键是否按下：0 为没按下；1 为按下
  int8_t right_button_down;  // 鼠标右键是否按下：0 为没按下，1 为按下
  uint8_t KEY_W : 1;         // W 键,0 为没按下，1 为按下
  uint8_t KEY_S : 1;         // s 键,0 为没按下，1 为按下
  uint8_t KEY_A : 1;         // A 键,0 为没按下，1 为按下
  uint8_t KEY_D : 1;         // D 键,0 为没按下，1 为按下
  uint8_t KEY_Shift : 1;     // Shift 键,0 为没按下，1 为按下
  uint8_t KEY_Ctrl : 1;      // Ctrl 键,0 为没按下，1 为按下
  uint8_t KEY_Q : 1;         // Q 键,0 为没按下，1 为按下
  uint8_t KEY_E : 1;         // E 键,0 为没按下，1 为按下
  uint8_t KEY_R : 1;         // R 键,0 为没按下，1 为按下
  uint8_t KEY_F : 1;         // F 键,0 为没按下，1 为按下
  uint8_t KEY_G : 1;         // G 键,0 为没按下，1 为按下
  uint8_t KEY_Z : 1;         // Z 键,0 为没按下，1 为按下
  uint8_t KEY_X : 1;         // X 键,0 为没按下，1 为按下
  uint8_t KEY_C : 1;         // C 键,0 为没按下，1 为按下
  uint8_t KEY_V : 1;         // V 键,0 为没按下，1 为按下
  uint8_t KEY_B : 1;         // B 键,0 为没按下，1 为按下
  uint16_t reserved;
} referee_remote_control_t;

extern game_state_t game_state;                              // 0x0001
extern game_result_t game_result;                            // 0x0002
extern game_robot_HP_t game_robot_HP;                        // 0x0003
extern event_data_t event_data;                              // 0x0101
extern supply_projectile_action_t supply_projectile_action;  // 0x0102
extern referee_warning_t referee_warning;                    // 0x0104
extern dart_remaining_time_t dart_remaining_time;            // 0x0105
extern robot_state_t robot_state;                            // 0x0201
extern power_heat_data_t power_heat_data;                    // 0x0202
extern game_robot_pos_t game_robot_pos;                      // 0x0203
extern robot_buff_t robot_buff;                              // x0204
extern air_support_data_t air_support_data;                  // 0x0205
extern hurt_data_t hurt_data;                                // 0x0206
extern shoot_data_t shoot_data;                              // 0x0207
extern projectile_allowance_t projectile_allowance;          // 0x0208
extern rfid_status_t rfid_status;                            // 0x0209
extern dart_client_cmd_t dart_client_cmd;                    // 0x020A
extern ground_robot_position_t ground_robot_position;        // 0x020B
extern radar_mark_data_t radar_mark_data;                    // 0x020C
extern robot_interaction_data_t robot_interaction_data;      // 0x0301
extern referee_remote_control_t referee_remote_control;      // 0x0304

#endif
