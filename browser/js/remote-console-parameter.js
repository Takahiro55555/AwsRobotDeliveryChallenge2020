const GAME_MODE = {
    "main": "GAME_MODE_IS_MAIN",  // 本戦モード
    "final": "GAME_MODE_IS_FINAL"  // 決勝戦モード
};

// ロボットがとりうる状態
const STATUS_DICT = {
    "initializing": { "status": "initializing", "msg": "初期化処理中 >>>走行不能<<<" },
    "ready": { "status": "ready", "msg": "走行可能（スタート可能）" },
    "running": { "status": "running", "msg": "走行中（停止・手動操作可能）" },
    "delivery": { "status": "delivery", "msg": "配達ポイントへ到着（リスタート・手動操作可能）" },
    "goal": { "status": "goal", "msg": "ゴールへ到着（リトライ・手動操作可能）" },
    "stop": { "status": "stop", "msg": "停止中（リスタート・手動操作可能）" },
    "manual": { "status": "manual", "msg": "手動操作中（リスタート可能）" },
    "error": { "status": "error", "msg": "エラー発生 >>>走行不能<<<" },
    "terminating": { "status": "terminating", "msg": "終了中 >>>走行不能<<<" }
};

// NOTE: この辞書のkeyは、ROS側のノード「remote_console」の「RemoteConsole::__data_publish_funcs」のkeyと一致させること
const subscribeTopics = {
    "globalCostmap": iotclientId + "/ros_to_remote_console/global_costmap/trimed",
    "localCostmap": iotclientId + "/ros_to_remote_console/local_costmap/trimed",
    "odom": iotclientId + "/ros_to_remote_console/odom",
    "mapGraph": iotclientId + "/ros_to_remote_console/planner/map_graph",
    "path": iotclientId + "/ros_to_remote_console/planner/path",
    "currentStatus": iotclientId + "/ros_to_remote_console/remote_console/current_status",
};

const publishTopics = {
    "buttons": iotclientId + "/remote_console_to_ros/buttons",
    "requestData": iotclientId + "/remote_console_to_ros/request_data",
};