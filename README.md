# AwsRobotDeliveryChallenge

## これはなに？
宮崎大学チームの、AwsRobotDeliveryChallengeのソースコードです。  

## 各ノードについて

### twugo_method
ロボットの制御をするノードです。

### commander
スタートの検知、目標座標の送信をするノードです。

### planner
経路計算をするノードです。

### img_proc
カメラに映った画像を表示するノードです。（現在未使用）

## 設定ファイルに関して
設定ファイルは以下のディレクトリにあります。
```
robot_ws/delivery_robot_sample/settings/
```

### map_graph.yaml
障害物が無い状態のマップ情報を設定しています。

### step.yaml
予選で使用した走行ルートを設定しています。

## メンバー紹介
* [Takahiro55555](https://github.com/Takahiro55555)
* [twugo](https://github.com/twugo)
* [nssuperx](https://github.com/nssuperx)
