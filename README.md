# AwsRobotDeliveryChallenge

## これはなに？
宮崎大学チームの、AwsRobotDeliveryChallengeのソースコードです。  

## Web UI の起動方法

### 前提条件
- docker をインストール済み
- `docker-compose` が使用可能

### 起動

```
cd AwsRobotDeliveryChallenge  # このリポジトリのルートディレクトリへ移動
docker-compose up  # Webサーバの起動(Apache)
```

#### **!! `browser` ディレクトリ以外をドキュメントルートにしたい場合**
デフォルトでは、このリポジトリの`browser`ディレクトリが HTTPサーバ のドキュメントルートになります。  
別のディレクトリをドキュメントルートに設定したい場合は、以下のコマンドの`[path to target dir]`部分を指定したいディレクトリへのパスに置き換えてください。  
パスは、プロジェクトルートからの相対パスでも、絶対パスでも大丈夫です。

毎回パスを指定するのが面倒な場合は、`.env`ファイルを編集してください。

```
AWS_WEB_UI_DIR="[path to target dir]" docker-compose up  # Webサーバの起動(Apache)
```

#### **!! ポート番号を指定したい場合**
デフォルトでは、`8080`番ポートを使用してHTTPサーバへアクセスするようにしています。  
別のポートを指定したい場合は、以下のコマンドの`[port number]`部分を指定したいポート番号に置き換えてください。  

毎回ポート番号を指定するのが面倒な場合は、`.env`ファイルを編集してください。

```
AWS_WEB_UI_PORT="[port number]" docker-compose up  # Webサーバの起動(Apache)
# AWS_WEB_UI_PORT="[port number]" AWS_WEB_UI_DIR="[path to target dir]" docker-compose up  # ポート番号とドキュメントルートを両方とも指定する場合
```

### WebUI へのアクセス

以下のリンクへ移動すると、WebUIにアクセスできます。  
[http://localhost:8080/](http://localhost:8080/)


## 各ノードについて

### twugo_method
ロボットの制御をするノードです。

### commander
スタートの検知、目標座標の送信をするノードです。

### planner
経路計算をするノードです。

### obstacle_detector
障害物の検知などを行います。


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
