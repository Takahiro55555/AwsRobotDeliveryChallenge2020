const awsIot = require('aws-iot-device-sdk');

const subscribeTopics = {};
subscribeTopics.costmap = iotclientId + "/ros_to_remote_console/obstacle_detector/merged_costmap/trimed";
subscribeTopics.odom = iotclientId + "/ros_to_remote_console/odom";
subscribeTopics.mapGraph = iotclientId + "/ros_to_remote_console/planner/map_graph";

const publishTopics = {};
publishTopics.gm = "gm_" + publish_topic;
publishTopics.buttons = iotclientId + "/remote_console_to_ros/buttons"

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
let odom = null;
let mapGraph = null;
let deviceIot = null;
async function setupAwsIot() {
    const credentials = await getCognitoCredentials();
    deviceIot = awsIot.device({
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
        if (topic == subscribeTopics.costmap) {
            costmap = JSON.parse(payload.toString());
            backgroundLayerP5.redraw();
            frontLayerP5.redraw();
        } else if (topic == subscribeTopics.odom) {
            odom = JSON.parse(payload.toString());
            frontLayerP5.redraw();
        } else if (topic == subscribeTopics.mapGraph) {
            mapGraph = JSON.parse(payload.toString());
            backgroundLayerP5.redraw();
        } else {
            console.log("[" + topic + "]" + JSON.parse(payload));
        }

        /**** 各種ボタンの有効化・無効化処理 ****/
        document.getElementById("btn-start-restart").removeAttribute("disabled");
        document.getElementById("btn-stop").removeAttribute("disabled");
    });

    // Subscribe する Topic を登録する
    for (key in subscribeTopics) {
        deviceIot.subscribe(subscribeTopics[key], undefined, function (err, granted) {
            // NOTE: iotclientId は極力表示しないほうが良いと考えたため、置換
            const blindedTopic = granted[0].topic.replace(iotclientId, "[AWS IoT thing name(replaced)]");
            console.log("Topic: " + blindedTopic);
            if (err) {
                console.log('subscribe error: ' + err);
            } else {
                console.log('subscribe success');
            }
        });
    }
}

/**** 以下、描画関係 ****/

let consoleWidth = document.getElementById("console").clientWidth;
let consoleHeight = consoleWidth / 130 * 70;

const background_layer_sketch = function (p) {
    p.preload = function () {
        p.VERTEX_ID_FONT = p.loadFont('/font/Roboto-Black.ttf');
    };

    p.setup = function () {
        p.createCanvas(consoleWidth, consoleHeight);
        p.background(200);
        p.noLoop();
    };

    p.draw = function () {
        /*** 以下、初期化処理 ***/
        p.background(255);

        /*** 以下、描画処理 ***/
        drawCostMap(p);
        if (costmap != null && mapGraph != null) {
            drawMapGraph(p, mapGraph, costmap, cellSize);
        }
    };
};


const front_layer_sketch = function (p) {
    p.preload = function () {
        // NOTE: p.loadImage 関数が Fetch API を使用しているため、
        // "img/TURTLEBOT3_BURGER.svg"を直接読み込もうとすると CORS 関連のエラーが発生した
        p.turtleBot3Image = p.loadImage("/img/TURTLEBOT3_BURGER.svg");
    };

    p.setup = function () {
        p.createCanvas(consoleWidth, consoleHeight, p.WEBGL);
        p.background(0, 0, 0, 0);
        p.noLoop();
    };

    p.draw = function () {
        /*** 以下、初期化処理 ***/
        p.clear();

        /*** 以下、描画処理 ***/
        if (odom != null && costmap != null) {
            drawTurtleBot3(p, odom, costmap, cellSize, consoleWidth, consoleHeight);
        }
    };
};

let cellSize = null;
function drawCostMap(p) {
    if (costmap === null) {
        return;
    }
    p.noStroke();
    const colorFrom = p.color(255);
    const colorTo = p.color(0);

    let w = consoleWidth / costmap.info.width;
    let h = consoleHeight / costmap.info.height;
    cellSize = w < h ? w : h;

    for (let row = 0; row < costmap.info.height; row++) {
        for (let col = 0; col < costmap.info.width; col++) {
            p.fill(p.lerpColor(colorFrom, colorTo, costmap.data[row][col] / 100));
            p.rect(col * cellSize, row * cellSize, cellSize, cellSize);
        }
    }
}

function drawMapGraph(p, mapGraph, costmap, cellSize) {
    const resolution = costmap.info.resolution;
    const linkedVertexsFlag = {};

    // Edge の描画
    for (key in mapGraph) {
        const x0 = cellSize / resolution * (mapGraph[key].x - costmap.info.origin.position.x);
        const y0 = cellSize / resolution * (mapGraph[key].y - costmap.info.origin.position.y);
        for (i in mapGraph[key].linked_vertex_list) {
            const linkedVertexId = mapGraph[key].linked_vertex_list[i];
            let linkedVertexListKey = String(key) + "-" + String(linkedVertexId);
            if (linkedVertexsFlag[linkedVertexListKey] === true) {
                continue;
            }
            const x1 = cellSize / resolution * (mapGraph[linkedVertexId].x - costmap.info.origin.position.x);
            const y1 = cellSize / resolution * (mapGraph[linkedVertexId].y - costmap.info.origin.position.y);

            p.stroke("#00ff00");
            p.strokeWeight(4);
            p.line(x0, y0, x1, y1);

            // フラグをセット
            linkedVertexsFlag[linkedVertexListKey] = true;
            linkedVertexListKey = String(linkedVertexId) + "-" + String(key);
            linkedVertexsFlag[linkedVertexListKey] = true;
        }
    }
    // Vertex の描画
    for (key in mapGraph) {
        const x0 = cellSize / resolution * (mapGraph[key].x - costmap.info.origin.position.x);
        const y0 = cellSize / resolution * (mapGraph[key].y - costmap.info.origin.position.y);

        p.stroke("#00bfff");
        p.strokeWeight(2);
        p.fill("#00bfff");
        p.circle(x0, y0, cellSize * 3);

        p.fill(0);
        p.textFont(p.VERTEX_ID_FONT);
        p.textSize(cellSize * 2);
        p.textAlign(p.CENTER, p.CENTER);
        p.text(key, x0, y0);
    }
}

function drawTurtleBot3(p, odom, costmap, cellSize, consoleWidth, consoleHeight) {
    const TURTLEBOT3_BURGER_REAL_SIZE_M = 0.21;
    const resolution = costmap.info.resolution;
    const trutlebot3Px = cellSize / resolution * TURTLEBOT3_BURGER_REAL_SIZE_M;
    const x = cellSize / resolution * (odom.pose.pose.position.x - costmap.info.origin.position.x);
    const y = cellSize / resolution * (odom.pose.pose.position.y - costmap.info.origin.position.y);
    const eulerOrientation = quaternionToEuler(odom.pose.pose.orientation);

    p.push();
    p.translate(-consoleWidth / 2 + x, -consoleHeight / 2 + y);  // 原点をロボットの位置にする
    // TODO: costmap の原点の orientation も考慮するようにする
    p.rotateZ(p.PI * 3 / 2 + eulerOrientation.z);  // ロボットの向きが実際の向きと一致するように座標系を
    p.imageMode(p.CENTER);  // 画像の基準点を画像中心部に設定
    p.image(p.turtleBot3Image, 0, 0, trutlebot3Px, trutlebot3Px);
    p.pop();
}

function quaternionToEuler(quat) {
    /**
     * ## 参考
     * - [ソースコード](https://www.npmjs.com/package/quaternion-to-euler)
     * - [文献](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)
     */
    if (!(Number.isFinite(quat.w) && Number.isFinite(quat.x) && Number.isFinite(quat.y) && Number.isFinite(quat.z))) {
        return { x: 0, y: 0, z: 0 };
    }
    const x = Math.atan2(2 * (quat.w * quat.x + quat.y * quat.z), 1 - (2 * (quat.x * quat.x + quat.y * quat.y)));
    const y = Math.asin(2 * (quat.w * quat.y - quat.z * quat.x));
    const z = Math.atan2(2 * (quat.w * quat.z + quat.x * quat.y), 1 - (2 * (quat.y * quat.y + quat.z * quat.z)));
    return { x, y, z };
};


/* AWS IoT へ接続する */
setupAwsIot();

/**** 以下、ボタン関係 ****/
document.getElementById("btn-start-restart").onclick = function startRestartButton() {
    if (deviceIot === null) {
        return;
    }
    const isStarted = this.value != "start";

    let payload = {};
    console.log("Game start!");
    request_id = (new Date()).getTime();
    payload["command"] = "game";
    payload["request_id"] = request_id

    if (!isStarted) {
        payload["action"] = "start";
        deviceIot.publish(publishTopics.gm, JSON.stringify(payload));
        this.innerHTML = "リスタート";
        this.value = "restart";
        this.classList.remove("btn-primary");
        this.classList.add("btn-success");
        return;
    }
    payload["action"] = "restart";
    deviceIot.publish(publishTopics.gm, JSON.stringify(payload));
}

document.getElementById("btn-stop").onclick = function stopButton() {
    if (deviceIot === null) {
        return;
    }
    let payload = {};
    request_id = (new Date()).getTime();

    payload["buttonName"] = "btn-stop";
    payload["request_id"] = request_id
    payload["isClicked"] = true;

    deviceIot.publish(publishTopics.buttons, JSON.stringify(payload));
}

function toggleCard(obj, id) {
    const pulusBtnPath = "/img/icon/plus.svg";
    const minusBtnPath = "/img/icon/minus.svg";
    const target = document.getElementById(id);

    const isHidden = target.hasAttribute("hidden");
    if (isHidden) {
        // 非表示部分を表示する
        target.removeAttribute("hidden");
        obj.src = minusBtnPath;
    } else {
        // 表示部分を非表示にする
        target.setAttribute("hidden", true);
        obj.src = pulusBtnPath;
    }
}

/* Sketch を DOM に追加 */
const backgroundLayerP5 = new p5(background_layer_sketch, "background-layer");
const frontLayerP5 = new p5(front_layer_sketch, "front-layer");
const backgroundLayerParent = document.getElementById("background-layer");
const frontLayerParent = document.getElementById("front-layer");
const consoleElement = document.getElementById("console");

function adjustCanvas() {
    consoleWidth = consoleElement.clientWidth;
    consoleHeight = Math.floor(consoleWidth / 130 * 70);

    /* Canvas の上位Element の高さを Canvas の高さと一緒にする */
    consoleElement.style.height = String(consoleHeight) + "px";
    backgroundLayerParent.style.height = String(consoleHeight) + "px";
    backgroundLayerParent.style.width = String(consoleWidth) + "px";
    frontLayerParent.style.height = String(consoleHeight) + "px";
    frontLayerParent.style.width = String(consoleWidth) + "px";

    /* Canvas サイズの変更 */
    backgroundLayerP5.resizeCanvas(consoleWidth, consoleHeight);
    frontLayerP5.resizeCanvas(consoleWidth, consoleHeight);
}

adjustCanvas();
window.onresize = adjustCanvas;