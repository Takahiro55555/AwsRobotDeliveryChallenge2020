const awsIot = require('aws-iot-device-sdk');

const subscribeTopics = {};
subscribeTopics.costmap = iotclientId + "/ros_to_remote_console/obstacle_detector/merged_costmap/trimed";
subscribeTopics.odom = iotclientId + "/ros_to_remote_console/odom";

async function getCognitoCredentials() {
    AWS.config.region = region;
    var cognitoidentity = new AWS.CognitoIdentity();
    var params = {
        IdentityPoolId: PoolId
    };
    const identityId = await cognitoidentity.getId(params).promise();
    const data = await cognitoidentity.getCredentialsForIdentity(identityId).promise();
    var credentials = {
        accessKey: data.Credentials.AccessKeyId,
        secretKey: data.Credentials.SecretKey,
        sessionToken: data.Credentials.SessionToken
    };
    return credentials;
}

let costmap = null;
async function setupAwsIot() {
    const credentials = await getCognitoCredentials();
    const deviceIot = awsIot.device({
        region: region,

        // FIXME: Client ID をランダムにすると、複数コンソールの同時接続という問題を考えないといけなくなる...
        clientId: iotclientId + "_" + String(Math.floor(Math.random() * 0x7ffffff)),
        accessKeyId: credentials.accessKey,
        secretKey: credentials.secretKey,
        sessionToken: credentials.sessionToken,
        protocol: 'wss',
        port: 443,
        host: iotendpoint
    });

    // メッセージ到着時の処理
    deviceIot.on('message', function (topic, payload) {
        console.log(topic);
        if (topic == subscribeTopics.costmap) {
            costmap = JSON.parse(payload.toString());
            isBackgroundLayerUpdated = true;
        } else if (topic == subscribeTopics.odom) {
            console.log(JSON.parse(payload.toString()));
        }
    });

    // Subscribe する Topic を登録する
    for (key in subscribeTopics) {
        deviceIot.subscribe(subscribeTopics[key], undefined, function (err, granted) {
            console.log("Topic: " + granted[0].topic);
            if (err) {
                console.log('subscribe error: ' + err);
            } else {
                console.log('subscribe success');
            }
        });
    }
}

/**** 以下、描画関係 ****/

const consoleWidth = document.getElementById("console").clientWidth;
const consoleHeight = 600;

let isBackgroundLayerUpdated = true;
const background_layer_sketch = function (p) {
    p.setup = function () {
        p.createCanvas(consoleWidth, consoleHeight);
        p.background(200);
    };

    p.draw = function () {
        if (!isBackgroundLayerUpdated) {
            return;
        }
        isBackgroundLayerUpdated = false;
        p.background(255);

        /*** 以下、描画処理 ***/
        drawCostMap(p);
    };
};


let isFrontLayerUpdated = true;
const front_layer_sketch = function (p) {
    p.setup = function () {
        p.createCanvas(consoleWidth, consoleHeight);
        p.background(0, 0, 0, 0);
    };

    p.draw = function () {
        if (!isFrontLayerUpdated) {
            return;
        }
        isFrontLayerUpdated = false;
        p.setup();

        /*** 以下、描画処理 ***/
    };
};


function drawCostMap(p) {
    if (costmap === null) {
        return;
    }
    p.noStroke();
    const colorFrom = p.color(255);
    const colorTo = p.color(0);

    let w = Math.floor(consoleWidth / costmap.info.width);
    let h = Math.floor(consoleHeight / costmap.info.height);
    const cellSize = w < h ? w : h;

    for (let row = 0; row < costmap.info.height; row++) {
        for (let col = 0; col < costmap.info.width; col++) {
            p.fill(p.lerpColor(colorFrom, colorTo, costmap.data[row][col] / 100));
            p.rect(col * cellSize, row * cellSize, cellSize, cellSize);
        }
    }
}


/* AWS IoT へ接続する */
setupAwsIot();

/* Sketch を DOM に追加 */
new p5(background_layer_sketch, "background-layer");
new p5(front_layer_sketch, "front-layer");
