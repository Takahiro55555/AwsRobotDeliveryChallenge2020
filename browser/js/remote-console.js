const awsIot = require('aws-iot-device-sdk');

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

let mergedCostmap = null;
let globalCostmap = null;
let localCostmap = null;
let isDrawGlobalCostmap = true;
let isDrawLocalCostmap = true;
let odom = null;
let originalMapGraph = null;
let editMapGraph = null;
let deviceIot = null;
let currentStatus = STATUS_DICT.initializing;
let gameMode = GAME_MODE.main;
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
        const msg = JSON.parse(payload.toString());
        if (topic == subscribeTopics.globalCostmap) {
            globalCostmap = msg;
            backgroundLayerP5.redraw();
            middleLayerP5.redraw();
            frontLayerP5.redraw();
        } else if (topic == subscribeTopics.localCostmap) {
            localCostmap = msg;
            backgroundLayerP5.redraw();
            middleLayerP5.redraw();
            frontLayerP5.redraw();
        } else if (topic == subscribeTopics.odom) {
            odom = msg;
            frontLayerP5.redraw();
        } else if (topic == subscribeTopics.mapGraph) {
            if (JSON.stringify(originalMapGraph) === JSON.stringify(editMapGraph)) {
                editMapGraph = msg;
            }
            originalMapGraph = msg;
            middleLayerP5.redraw();
        } else if (topic == subscribeTopics.currentStatus) {
            /**** 各種ボタンの有効化・無効化処理 ****/
            if (!("status" in msg)) {
                return;
            }
            if ("gameMode" in msg) {
                applyGameMode(msg.gameMode);
            }
            const recievedStatus = msg["status"];
            if (recievedStatus === STATUS_DICT.goal.status) {
                document.getElementById("btn-retry-game").removeAttribute("disabled");
                document.getElementById("btn-stop").removeAttribute("disabled");
                makeRestartButton();
            } else if (recievedStatus === STATUS_DICT.ready.status) {
                document.getElementById("btn-retry-game").setAttribute("disabled", true);
                document.getElementById("btn-stop").setAttribute("disabled", true);
                makeStartButton();
                makeToEnableModeSelect();
            } else if (recievedStatus === STATUS_DICT.running.status || recievedStatus === STATUS_DICT.stop.status || recievedStatus === STATUS_DICT.delivery.status || recievedStatus === STATUS_DICT.manual.status) {
                document.getElementById("btn-stop").removeAttribute("disabled");
                makeRestartButton();
                makeToDisableModeSelect();
                if (document.getElementById("btn-start-restart").value == "start") {
                    document.getElementById("btn-retry-game").setAttribute("disabled", true);
                }
            }
            if ("isGoto" in msg && msg.isGoto) {
                document.getElementById("btn-stop").removeAttribute("disabled");
                currentGotoPoint = msg.point;
                middleLayerP5.redraw();
            } else {
                currentGotoPoint = null;
                middleLayerP5.redraw();
            }
            currentStatus = STATUS_DICT[recievedStatus];
        } else {
            console.log("[" + topic + "] Message recieved.");
        }
    });

    // Subscribe する Topic を登録する
    for (key in subscribeTopics) {
        deviceIot.subscribe(subscribeTopics[key], undefined, function (err, granted) {
            // NOTE: iotclientId は極力表示しないほうが良いと考えたため、置換
            const blindedTopic = granted[0].topic.replace(iotclientId, "[IoTClientId(replaced)]");
            console.log("Topic: " + blindedTopic);
            if (err) {
                console.log('subscribe error: ' + err);
            } else {
                console.log('subscribe success');
            }
        });
    }
    const payload = { 'requestDataList': ['all'] };
    deviceIot.publish(publishTopics.requestData, JSON.stringify(payload));
}

/**** 以下、描画関係 ****/

let consoleWidth = document.getElementById("console").clientWidth;
let consoleHeight = consoleWidth / 130 * 70;
let isDrawGotoPoint = false;
let currentGotoPoint = null;
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
        drawCostmap(p);
    };
};


const middle_layer_sketch = function (p) {
    p.preload = function () {
        p.VERTEX_ID_FONT = p.loadFont('/font/Roboto-Black.ttf');
    };

    p.setup = function () {
        p.createCanvas(consoleWidth, consoleHeight);
        p.background(0, 0, 0, 0);
        p.noLoop();
    };

    p.draw = function () {
        /*** 以下、初期化処理 ***/
        p.clear();

        /*** 以下、描画処理 ***/
        if (mergedCostmap != null && originalMapGraph != null) {
            drawMapGraph(p, originalMapGraph, mergedCostmap, cellSize);
        }

        if (isDrawGotoPoint && mergedCostmap != null) {
            const x = document.getElementById("number-goto-coordinate-x").value;
            const y = document.getElementById("number-goto-coordinate-y").value;
            const tolerance = document.getElementById("number-goto-tolerance").value;
            if (isNaN(x) || isNaN(y) || isNaN(tolerance) || x === "" || y === "" || tolerance === "") {
                isDrawGotoPoint = false;
                document.getElementById("btn-goto").setAttribute("disabled", true);
                return
            }
            drawTemporaryGotoPoint(p, x, y, tolerance, mergedCostmap.info.origin, mergedCostmap.info.resolution, cellSize);
        }
        if (mergedCostmap != null && currentGotoPoint != null) {
            drawCurrentGotoPoint(p, currentGotoPoint.x, currentGotoPoint.y, currentGotoPoint.tolerance, mergedCostmap.info.origin, mergedCostmap.info.resolution, cellSize);
        }
    };
}

let activeVertexId = null;
let unlinkedVertexList = [];
let linkedVertexList = [];
const front_layer_sketch = function (p) {
    p.preload = function () {
        // NOTE: p.loadImage 関数が Fetch API を使用しているため、
        // "img/TURTLEBOT3_BURGER.svg"を直接読み込もうとすると CORS 関連のエラーが発生した
        p.turtleBot3Image = p.loadImage("/img/TURTLEBOT3_BURGER.svg");
    };

    p.setup = function () {
        p.createCanvas(consoleWidth, consoleHeight, p.WEBGL).mouseClicked(mouseClicked);
        p.background(0, 0, 0, 0);
        p.noLoop();
    };

    p.draw = function () {
        /*** 以下、初期化処理 ***/
        p.clear();

        /*** 以下、描画処理 ***/
        if (odom != null && mergedCostmap != null) {
            drawTurtleBot3(p, odom, mergedCostmap, cellSize, consoleWidth, consoleHeight);
        }
    };

    function mouseClicked(obj) {
        // Vertex をクリックしているかどうか判定
        for (let i = 0; i < vertexIdListOnCanvas.length; i++) {
            const vertexId = vertexIdListOnCanvas[i];
            const x0 = vertexCoordinateDictOnCanvas[vertexId].x0;
            const y0 = vertexCoordinateDictOnCanvas[vertexId].y0;
            const distance = ((x0 - p.mouseX) ** 2 + (y0 - p.mouseY) ** 2) ** 0.5;
            if (distance <= cellSize * VERTEX_DIAMETER_MAGNIFICATION_FROM_CELL_SIZE / 2) {
                const cardConsoleEditorElm = document.getElementById("card-console-editor");
                cardConsoleEditorElm.style.top = String(obj.pageY + 50) + "px";
                cardConsoleEditorElm.style.left = String(obj.pageX + 50) + "px";
                cardConsoleEditorElm.removeAttribute("hidden");
                activeVertexId = vertexId;
                linkedVertexList.length = 0;
                unlinkedVertexList.length = 0;
                linkedVertexList = editMapGraph[activeVertexId].linked_vertex_list.concat([]);  // 参照渡しを防ぐ
                for (let key in editMapGraph) {
                    if (Number(editMapGraph[key].id) === Number(activeVertexId)) {
                        continue;
                    }
                    if (!linkedVertexList.includes(Number(editMapGraph[key].id))) {
                        unlinkedVertexList.push(Number(editMapGraph[key].id));
                    }
                }
                setDataOnVertexEditor(activeVertexId, editMapGraph[activeVertexId], linkedVertexList, unlinkedVertexList);
                middleLayerP5.redraw();
                return;
            }
        }

        // GoTo を設定
    }
};

/**
 * 
 * @param {Object} p p5.js object renderer は p5.P2Dであるという前提
 * @param {number} x goto する座標 [m]
 * @param {number} y goto する座標 [m]
 * @param {number} tolerance [m]
 * @param {Object} origin 描画されているMapの実世界における原点座標[m]
 * @param {number} resolution Map1ピクセル当たりの実世界における大きさ[m]
 * @param {number} cellSize Canvas上で描画する際のMap1ピクセル当たりの大きさ[pix]
 * @param {string} strokeColor 表示する際の色
 */
function drawTemporaryGotoPoint(p, x, y, tolerance, origin, resolution, cellSize, strokeColor = "#000") {
    const x0 = cellSize / resolution * (x - origin.position.x);
    const y0 = cellSize / resolution * (y - origin.position.y);
    const circleRadius = cellSize / resolution * tolerance;

    p.push();
    p.drawingContext.setLineDash([5, 5]);
    p.noFill();
    p.stroke(strokeColor);
    p.strokeWeight(1);
    p.line(x0 - circleRadius, y0, x0 + circleRadius, y0);
    p.line(x0, y0 - circleRadius, x0, y0 + circleRadius);
    p.strokeWeight(2);
    p.circle(x0, y0, circleRadius * 2);
    p.pop();
}

function drawCurrentGotoPoint(p, x, y, tolerance, origin, resolution, cellSize, strokeColor = "#ff4500") {
    const x0 = cellSize / resolution * (x - origin.position.x);
    const y0 = cellSize / resolution * (y - origin.position.y);
    const circleRadius = cellSize / resolution * tolerance;

    p.push();
    p.noFill();
    p.stroke(strokeColor);
    p.strokeWeight(2);
    p.line(x0 - circleRadius, y0, x0 + circleRadius, y0);
    p.line(x0, y0 - circleRadius, x0, y0 + circleRadius);
    p.strokeWeight(3);
    p.circle(x0, y0, circleRadius * 2);
    p.pop();
}

function applyIndicateCostmap(_isDrawGlobalCostmap, _isDrawLocalCostmap) {
    if (typeof (_isDrawGlobalCostmap) !== "boolean") {
        return;
    }
    if (typeof (_isDrawLocalCostmap) !== "boolean") {
        return;
    }
    isDrawGlobalCostmap = _isDrawGlobalCostmap;
    isDrawLocalCostmap = _isDrawLocalCostmap;

    backgroundLayerP5.redraw();
    middleLayerP5.redraw();
    frontLayerP5.redraw();
}

let cellSize = null;
function drawCostmap(p) {
    if (localCostmap === null && globalCostmap === null) {
        return;
    }
    if (isDrawGlobalCostmap && globalCostmap === null) {
        return;
    }
    if (isDrawLocalCostmap && localCostmap === null) {
        return
    }
    const globalColorFrom = p.color(255, 255, 255, 0);
    const globalColorTo = p.color(0, 0, 0, 180);
    const localColorFrom = p.color(255, 255, 255, 0);
    const localColorTo = p.color(0, 0, 255, 180);
    let colorList = null;
    if (isDrawGlobalCostmap && isDrawLocalCostmap) {
        mergedCostmap = mergeCostmaps([globalCostmap, localCostmap]);
        colorList = [{ "from": globalColorFrom, "to": globalColorTo }, { "from": localColorFrom, "to": localColorTo }];
    } else if (isDrawGlobalCostmap) {
        mergedCostmap = mergeCostmaps([globalCostmap]);
        colorList = [{ "from": globalColorFrom, "to": globalColorTo }];
    } else if (isDrawLocalCostmap) {
        mergedCostmap = mergeCostmaps([localCostmap]);
        colorList = [{ "from": localColorFrom, "to": localColorTo }];
    }
    p.noStroke();
    let w = consoleWidth / mergedCostmap.info.width;
    let h = consoleHeight / mergedCostmap.info.height;
    cellSize = w < h ? w : h;

    for (let i = 0; i < mergedCostmap.costmaps.length; i++) {
        for (let row = 0; row < mergedCostmap.costmaps[i].height; row++) {
            for (let col = 0; col < mergedCostmap.costmaps[i].width; col++) {
                p.fill(p.lerpColor(colorList[i].from, colorList[i].to, mergedCostmap.costmaps[i].data[row][col] / 100));
                p.rect((col + mergedCostmap.costmaps[i].indexPaddingX) * cellSize, (row + mergedCostmap.costmaps[i].indexPaddingY) * cellSize, cellSize, cellSize);
            }
        }
    }
}

// NOTE: 全てのcostmapのorigin.orientation が一致するという前提の関数
function mergeCostmaps(costmapList) {
    if (costmapList.length < 1) {
        return null;
    }
    const orientationString = JSON.stringify(costmapList[0].info.origin.orientation);
    const resolution = costmapList[0].info.resolution;
    let startX = costmapList[0].info.origin.position.x;
    let startY = costmapList[0].info.origin.position.y;
    let endX = startX + costmapList[0].info.width * resolution;
    let endY = startY + costmapList[0].info.height * resolution;
    for (let i = 1; i < costmapList.length; i++) {
        // 全てのcostmapのorigin.orientation が一致するという前提が崩れた場合は、nullを返す
        if (orientationString != JSON.stringify(costmapList[i].info.origin.orientation)) {
            console.log("Costmap 同士の Orientation が一致しませんでした [index: " + String(i) + "]");
            return null;
        }
        if (resolution != costmapList[i].info.resolution) {
            console.log("Costmap 同士の Resolution が一致しませんでした [index: " + String(i) + "]");
            return null;
        }
        const sx = costmapList[i].info.origin.position.x;
        const sy = costmapList[i].info.origin.position.y;
        const ex = sx + costmapList[i].info.width * resolution;
        const ey = sy + costmapList[i].info.height * resolution;

        if (sx < startX) {
            startX = sx;
        }
        if (sy < startY) {
            startY = sy;
        }
        if (ex > endX) {
            endX = ex;
        }
        if (ey > endY) {
            endY = ey;
        }
    }
    const mergedCostmap = { "info": { "origin": { "position": {} } } };
    mergedCostmap.info.origin.position.x = startX;
    mergedCostmap.info.origin.position.y = startY;
    mergedCostmap.info.resolution = resolution;
    mergedCostmap.info.width = Math.floor((endX - startX) / resolution);
    mergedCostmap.info.height = Math.floor((endY - startY) / resolution);
    mergedCostmap.costmaps = [];
    for (let i = 0; i < costmapList.length; i++) {
        const costmap = {};
        costmap.data = JSON.parse(JSON.stringify(costmapList[i].data));
        costmap.width = costmapList[i].info.width;
        costmap.height = costmapList[i].info.height;
        costmap.indexPaddingX = ((costmapList[i].info.origin.position.x - startX) / resolution);
        costmap.indexPaddingY = ((costmapList[i].info.origin.position.y - startY) / resolution);
        mergedCostmap.costmaps.push(costmap);
    }
    return mergedCostmap
}

const VERTEX_DIAMETER_MAGNIFICATION_FROM_CELL_SIZE = 3;
const edgeCoordinateListOnCanvas = [];  // クリック判定の高速化のために、キャンバス上の座標をキャッシュしておく
const vertexIdListOnCanvas = [];  // UI上の表示順序を保持する
const vertexCoordinateDictOnCanvas = {};  // クリック判定の高速化のために、キャンバス上の座標をキャッシュしておく
function drawMapGraph(p, mapGraph, costmap, cellSize) {
    const resolution = costmap.info.resolution;
    const linkedVertexsFlag = {};

    // Edge の描画
    edgeCoordinateListOnCanvas.length = 0;
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
            edgeCoordinateListOnCanvas.unshift({ x0, y0, x1, y1 });  // edge が重なった際、手前のほうの edge をクリック判定するために unshift を使用

            p.stroke("#afeeee");
            p.strokeWeight(4);
            p.line(x0, y0, x1, y1);

            // フラグをセット
            linkedVertexsFlag[linkedVertexListKey] = true;
            linkedVertexListKey = String(linkedVertexId) + "-" + String(key);
            linkedVertexsFlag[linkedVertexListKey] = true;
        }
    }
    // Vertex の描画
    vertexIdListOnCanvas.length = 0;
    for (key in mapGraph) {
        const x0 = cellSize / resolution * (mapGraph[key].x - costmap.info.origin.position.x);
        const y0 = cellSize / resolution * (mapGraph[key].y - costmap.info.origin.position.y);
        vertexIdListOnCanvas.unshift(key); // vertex が重なった際、手前のほうの vertex をクリック判定するために unshift を使用
        vertexCoordinateDictOnCanvas[key] = { x0, y0 };

        p.push();
        p.stroke("#87cefa");
        p.fill("#87cefa");
        if (key === activeVertexId) {
            p.stroke("#00ff00");
            p.fill("#00ff00");
        } else if (mapGraph[key].is_via_point || mapGraph[key].is_destination) {
            p.stroke("#ffd700");
            if (mapGraph[key].is_destination) {
                p.fill("#ffd700");
            }
        }
        p.strokeWeight(2);
        p.circle(x0, y0, cellSize * VERTEX_DIAMETER_MAGNIFICATION_FROM_CELL_SIZE);
        p.pop();

        p.push();
        p.strokeWeight(2);
        p.stroke("#ff00ff");
        p.noFill();
        p.circle(x0, y0, cellSize / resolution * mapGraph[key].tolerance * 2);
        p.pop();

        p.push();
        p.noStroke();
        p.fill(0);
        p.textFont(p.VERTEX_ID_FONT);
        p.textSize(cellSize * (VERTEX_DIAMETER_MAGNIFICATION_FROM_CELL_SIZE) * 0.6);
        p.textAlign(p.CENTER, p.CENTER);
        p.text(key, x0, y0);
        p.pop();
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
    requestId = (new Date()).getTime();
    payload["requestId"] = requestId
    payload["isClicked"] = true;

    if (!isStarted) {
        payload["buttonName"] = "btn-start";
        payload["gameMode"] = gameMode;
        deviceIot.publish(publishTopics.buttons, JSON.stringify(payload));
        makeToDisableModeSelect();
        return;
    }
    payload["buttonName"] = "btn-restart";
    deviceIot.publish(publishTopics.buttons, JSON.stringify(payload));
}

function makeStartButton() {
    const btnElm = document.getElementById("btn-start-restart");
    const START_BTN_CLASS = "btn btn-primary btn-lg btn-block btn-line-through-on-disabled";
    btnElm.innerHTML = "スタート";
    btnElm.value = "start";
    btnElm.setAttribute("class", START_BTN_CLASS);
    btnElm.removeAttribute("disabled");
}

function makeRestartButton() {
    const btnElm = document.getElementById("btn-start-restart");
    const RESTART_BTN_CLASS = "btn btn-success btn-lg btn-block btn-line-through-on-disabled";
    btnElm.innerHTML = "リスタート";
    btnElm.value = "restart";
    btnElm.setAttribute("class", RESTART_BTN_CLASS);
    btnElm.removeAttribute("disabled");
}

document.getElementById("btn-console-set-coordinate-and-tolerance-to-goto").onclick = setCoordinateAndToleranceFromVertexEditor;
function setCoordinateAndToleranceFromVertexEditor() {
    const x = document.getElementById("number-vertex-coordinate-x").value;
    const y = document.getElementById("number-vertex-coordinate-y").value;
    const tolerance = document.getElementById("number-vertex-tolerance").value;
    document.getElementById("number-goto-coordinate-x").value = x;
    document.getElementById("number-goto-coordinate-y").value = y;
    document.getElementById("number-goto-tolerance").value = tolerance;
    document.getElementById("number-goto-tolerance").dispatchEvent(new Event('input'));
}

document.getElementById("number-goto-coordinate-x").oninput = onInputGoto;
document.getElementById("number-goto-coordinate-y").oninput = onInputGoto;
document.getElementById("number-goto-tolerance").oninput = onInputGoto;
function onInputGoto() {
    const x = document.getElementById("number-goto-coordinate-x").value;
    const y = document.getElementById("number-goto-coordinate-y").value;
    const tolerance = document.getElementById("number-goto-tolerance").value;
    if (isNaN(x) || isNaN(y) || isNaN(tolerance) || x === "" || y === "" || tolerance === "") {
        if (isDrawGotoPoint) {
            isDrawGotoPoint = false;
            middleLayerP5.redraw();
        }
        document.getElementById("btn-goto").setAttribute("disabled", true);
        return
    }
    document.getElementById("btn-goto").removeAttribute("disabled");
    isDrawGotoPoint = true;
    middleLayerP5.redraw();
}

document.getElementById("btn-goto").onclick = requestGoto;
function requestGoto() {
    if (deviceIot === null) {
        return;
    }
    const x = Number(document.getElementById("number-goto-coordinate-x").value);
    const y = Number(document.getElementById("number-goto-coordinate-y").value);
    const tolerance = Number(document.getElementById("number-goto-tolerance").value);
    if (isNaN(x) || isNaN(y) || isNaN(tolerance) || x === "" || y === "" || tolerance === "") {
        isDrawGotoPoint = false;
        document.getElementById("btn-goto").setAttribute("disabled", true);
        return
    }
    let payload = {};
    requestId = (new Date()).getTime();
    payload["buttonName"] = "btn-goto";
    payload["requestId"] = requestId;
    payload["isClicked"] = true;
    payload["point"] = { "x": x, "y": y, "tolerance": tolerance };
    deviceIot.publish(publishTopics.buttons, JSON.stringify(payload));
}

document.getElementById("btn-stop").onclick = stopButton;
function stopButton() {
    if (deviceIot === null) {
        return;
    }
    let payload = {};
    requestId = (new Date()).getTime();
    payload["buttonName"] = "btn-stop";
    payload["requestId"] = requestId;
    payload["isClicked"] = true;
    deviceIot.publish(publishTopics.buttons, JSON.stringify(payload));
}

document.getElementById("btn-retry-game").onclick = retryGameButton;
function retryGameButton() {
    if (deviceIot === null) {
        return;
    }
    let payload = {};
    requestId = (new Date()).getTime();
    payload["buttonName"] = "btn-retry-game";
    payload["requestId"] = requestId;
    payload["isClicked"] = true;
    deviceIot.publish(publishTopics.buttons, JSON.stringify(payload));
}



document.getElementById("btn-apply-mode").onclick = applyGameMode;
function applyGameMode(selectedGameMode = null) {
    if (selectedGameMode === null || typeof (selectedGameMode) != typeof ("")) {
        selectedGameMode = document.getElementById("li-game-mode").value;
    }
    if (selectedGameMode === GAME_MODE.main) {
        document.getElementById("txt-current-game-mode").innerText = "本戦";
        document.getElementById("game-mode-option-main").selected = true;
    } else if (selectedGameMode === GAME_MODE.final) {
        document.getElementById("txt-current-game-mode").innerText = "決勝戦"
        document.getElementById("game-mode-option-final").selected = true;
    } else {
        return;
    }
    gameMode = selectedGameMode;
    document.getElementById("btn-apply-mode").setAttribute("disabled", true);
    document.getElementById("li-game-mode").value = gameMode;
}

document.getElementById("li-game-mode").onchange = function () {
    const selectedGameMode = document.getElementById("li-game-mode").value;
    if (selectedGameMode != gameMode) {
        document.getElementById("btn-apply-mode").removeAttribute("disabled");
    }
}

function makeToDisableModeSelect() {
    document.getElementById("btn-apply-mode").setAttribute("disabled", true);
    document.getElementById("li-game-mode").setAttribute("disabled", true);
    document.getElementById("li-game-mode").value = gameMode;
}

function makeToEnableModeSelect() {
    document.getElementById("li-game-mode").removeAttribute("disabled");
    document.getElementById("li-game-mode").value = gameMode;

    // 選択リストの値を変更しない限り、適用ボタンは無効化したままにしておく
    document.getElementById("btn-apply-mode").setAttribute("disabled", true);
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

function setDataOnVertexEditor(vertexId, vertex, linkedVertexIdList, unlinkedVertexIDList) {
    const BADGE_CLASS = "badge badge-primary badge-linked-vertex m-1";
    const BADGE_CLICK_HANDLER = "copyValue(this, 'number-linked-vertex-id')";
    let maxInputValue = null;
    let minInputValue = null;

    // タイトルの設定
    const cardTitle = "Vertex Editor (ID: " + String(vertexId) + " )";
    document.getElementById("txt-console-card-title").innerText = cardTitle;

    // チェックボックス（フラグ）の設定
    document.getElementById("checkbox-vertex-is-destination").checked = vertex.is_destination;
    document.getElementById("checkbox-vertex-is-via-point").checked = vertex.is_via_point;

    // 座標の設定
    document.getElementById("number-vertex-coordinate-x").value = vertex.x;
    document.getElementById("number-vertex-coordinate-y").value = vertex.y;

    // tolerance の設定
    document.getElementById("number-vertex-tolerance").value = vertex.tolerance;

    // Vertex 入力欄のクリア
    document.getElementById("number-linked-vertex-id").value = "";
    document.getElementById("number-linked-vertex-id").dispatchEvent(new Event('input'));

    // 入力 Vertex 候補の設定
    const linkedVertexDatalistParentElm = document.getElementById("datalist-vertex-id");
    while (linkedVertexDatalistParentElm.firstChild) {
        linkedVertexDatalistParentElm.removeChild(linkedVertexDatalistParentElm.firstChild);
    }
    if (unlinkedVertexIDList.length > 0) {
        maxInputValue = unlinkedVertexIDList[0];
        minInputValue = unlinkedVertexIDList[0];
    }
    for (let i = 0; i < unlinkedVertexIDList.length; i++) {
        const newOptionElm = document.createElement("option");
        newOptionElm.setAttribute("value", unlinkedVertexIDList[i]);
        linkedVertexDatalistParentElm.appendChild(newOptionElm);
        if (maxInputValue < unlinkedVertexIDList[i]) {
            maxInputValue = unlinkedVertexIDList[i];
        }
        if (minInputValue > unlinkedVertexIDList[i]) {
            minInputValue = unlinkedVertexIDList[i];
        }
    }

    // 隣接 Vertex の設定・入力　Vertex 候補の設定
    const linkedVertexBadgeParentElm = document.getElementById("linked-vertex-badge-list");
    while (linkedVertexBadgeParentElm.firstChild) {
        linkedVertexBadgeParentElm.removeChild(linkedVertexBadgeParentElm.firstChild);
    }
    if (linkedVertexIdList.length > 0 && maxInputValue === null && minInputValue === null) {
        maxInputValue = linkedVertexIdList[0];
        minInputValue = linkedVertexIdList[0];
    }
    for (let i = 0; i < linkedVertexIdList.length; i++) {
        const newOptionElm = document.createElement("option");
        newOptionElm.setAttribute("value", linkedVertexIdList[i]);
        linkedVertexDatalistParentElm.appendChild(newOptionElm);

        const newBadgeElm = document.createElement("button");
        newBadgeElm.innerText = linkedVertexIdList[i];
        newBadgeElm.value = linkedVertexIdList[i];
        newBadgeElm.setAttribute("class", BADGE_CLASS);
        newBadgeElm.setAttribute("onclick", BADGE_CLICK_HANDLER);
        linkedVertexBadgeParentElm.appendChild(newBadgeElm);
        if (maxInputValue < linkedVertexIdList[i]) {
            maxInputValue = linkedVertexIdList[i];
        }
        if (minInputValue > linkedVertexIdList[i]) {
            minInputValue = linkedVertexIdList[i];
        }
    }

    // Vertex 入力欄の最大値・最小値の設定
    document.getElementById("number-linked-vertex-id").setAttribute("max", String(maxInputValue));
    document.getElementById("number-linked-vertex-id").setAttribute("min", String(minInputValue));

}

function closeConsoleCard(id) {
    const target = document.getElementById(id);
    target.setAttribute("hidden", true);
    activeVertexId = null;
    unlinkedVertexList.length = 0;
    linkedVertexList.length = 0;
    middleLayerP5.redraw();
}

function updateLinkedVertexButton(obj) {

    const DISABLED_BTN_TEXT = "Add or Remove";
    const DISABLED_BTN_CLASS = "btn btn-outline-secondary btn-block btn-line-through-on-disabled";
    const ADD_BTN_TEXT = "Add";
    const ADD_BTN_CLASS = "btn btn-primary btn-block";
    const REMOVE_BTN_TEXT = "Remove";
    const REMOVE_BTN_CLASS = "btn btn-danger btn-block";
    const btnElm = document.getElementById("btn-add-remove-linked-vertex");
    const inputValue = obj.value;

    // とりあえずボタンを無効化しておく
    btnElm.setAttribute("disabled", true);
    btnElm.setAttribute("class", DISABLED_BTN_CLASS);
    btnElm.innerText = DISABLED_BTN_TEXT;

    // NOTE: String型へキャストしているのは、厳密等価演算子を使用するため
    // 入力された ID が activeVertex のものと一致したら、無効化したままにしておく
    if (String(inputValue) === String(activeVertexId)) {
        return;
    }
    if (String(inputValue) === "") {
        return;
    }
    // Linked vertex 追加又は削除ボタン更新
    if (unlinkedVertexList.includes(Number(inputValue))) {
        btnElm.removeAttribute("disabled");
        btnElm.setAttribute("class", ADD_BTN_CLASS);
        btnElm.innerText = ADD_BTN_TEXT;
    } else if (linkedVertexList.includes(Number(inputValue))) {
        btnElm.removeAttribute("disabled");
        btnElm.setAttribute("class", REMOVE_BTN_CLASS);
        btnElm.innerText = REMOVE_BTN_TEXT;
    }
}

function copyValue(obj, targetElmId) {
    const targetElm = document.getElementById(targetElmId);
    if (!obj.hasAttribute("value")) {
        console.error("ERROR: this Element does not have `value` attribute.");
        return;
    }
    if (!targetElm.hasAttribute("value")) {
        console.error("ERROR: target Element(" + targetElmId + ") does not have `value` attribute.");
        return;
    }
    targetElm.value = obj.value;
    targetElm.dispatchEvent(new Event('input'));
}

/* Sketch を DOM に追加 */
const backgroundLayerP5 = new p5(background_layer_sketch, "background-layer");
const middleLayerP5 = new p5(middle_layer_sketch, "middle-layer");
const frontLayerP5 = new p5(front_layer_sketch, "front-layer");
const backgroundLayerParent = document.getElementById("background-layer");
const middleLayerParent = document.getElementById("middle-layer");
const frontLayerParent = document.getElementById("front-layer");
const consoleElement = document.getElementById("console");

function adjustCanvas() {
    consoleWidth = consoleElement.clientWidth;
    consoleHeight = Math.floor(consoleWidth / 130 * 70);

    /* Canvas の上位Element の高さを Canvas の高さと一緒にする */
    consoleElement.style.height = String(consoleHeight) + "px";
    backgroundLayerParent.style.height = String(consoleHeight) + "px";
    backgroundLayerParent.style.width = String(consoleWidth) + "px";
    middleLayerParent.style.height = String(consoleHeight) + "px";
    middleLayerParent.style.width = String(consoleWidth) + "px";
    frontLayerParent.style.height = String(consoleHeight) + "px";
    frontLayerParent.style.width = String(consoleWidth) + "px";

    /* Canvas サイズの変更 */
    backgroundLayerP5.resizeCanvas(consoleWidth, consoleHeight);
    middleLayerP5.resizeCanvas(consoleWidth, consoleHeight);
    frontLayerP5.resizeCanvas(consoleWidth, consoleHeight);
}
adjustCanvas();
window.onresize = adjustCanvas;

/**** キャンバス右クリック時に独自メニューを出すための処理 ****/
frontLayerParent.addEventListener('contextmenu', function (e) {
    console.log("Right button pressed");
    if (document.getElementById("number-goto-tolerance").value === "") {
        document.getElementById("number-goto-tolerance").value = 0.12;
    }
    const x = Math.floor((mergedCostmap.info.origin.position.x + frontLayerP5.mouseX / cellSize * mergedCostmap.info.resolution) * 10000) / 10000;
    const y = Math.floor((mergedCostmap.info.origin.position.y + frontLayerP5.mouseY / cellSize * mergedCostmap.info.resolution) * 10000) / 10000;
    document.getElementById("number-goto-coordinate-x").value = x;
    document.getElementById("number-goto-coordinate-y").value = y;
    document.getElementById("number-goto-tolerance").dispatchEvent(new Event('input'));

    let yamlString = ""
    yamlString += "- id: ";
    yamlString += "\n    x: ";
    yamlString += String(x);
    yamlString += "\n    y: ";
    yamlString += String(y);
    yamlString += "\n    tolerance: ";
    yamlString += String(document.getElementById("number-goto-tolerance").value);
    yamlString += "\n    is_destination: false";
    yamlString += "\n    linked_vertex_list:";
    yamlString += "\n      - ";

    console.log(yamlString)
});

/**** 自由にドラッグして配置できるUIを実現するための処理 ****/
let beforeX = null;  // ドラッグ Event の一番最後の座標がいつも(0, 0)になってしまう問題を解消するために使用
let beforeY = null;  // ドラッグ Event の一番最後の座標がいつも(0, 0)になってしまう問題を解消するために使用
let offsetX = 0;  // ドラッグ Event 開始時の、対称 Element を基準としたマウスの位置を格納
let offsetY = 0;　// ドラッグ Event 開始時の、対称 Element を基準としたマウスの位置を格納
document.addEventListener("drag", function (event) {
    if (!event.target.hasAttribute("draggable")) {
        return;
    }
    if (beforeX === null || beforeY === null) {
        event.target.style.left = String(event.pageX - offsetX) + "px";
        event.target.style.top = String(event.pageY - offsetY) + "px";
    } else {
        event.target.style.left = beforeX;
        event.target.style.top = beforeY;
    }
    beforeX = String(event.pageX - offsetX) + "px";
    beforeY = String(event.pageY - offsetY) + "px";
}, false);

document.addEventListener("dragstart", function (event) {
    if (!event.target.hasAttribute("draggable")) {
        return;
    }
    offsetX = event.offsetX;
    offsetY = event.offsetY;
    event.target.style.opacity = 0.5;  // 不透明度の設定
}, false);

document.addEventListener("dragend", function (event) {
    if (!event.target.hasAttribute("draggable")) {
        return;
    }
    beforeX = null;
    beforeY = null;
    event.target.style.opacity = 1;  // 不透明度の設定
}, false);
