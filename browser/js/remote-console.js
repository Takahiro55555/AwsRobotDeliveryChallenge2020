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
let odom = null;
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
        if (topic == subscribeTopics.costmap) {
            costmap = JSON.parse(payload.toString());
            isBackgroundLayerUpdated = true;
            isFrontLayerUpdated = true;
        } else if (topic == subscribeTopics.odom) {
            odom = JSON.parse(payload.toString());
            isFrontLayerUpdated = true;
        }
    });

    // Subscribe する Topic を登録する
    for (key in subscribeTopics) {
        deviceIot.subscribe(subscribeTopics[key], undefined, function (err, granted) {
            // NOTE: iotclientId は極力表示しないほうが良いと考えたため、置換
            const blindedTopic = granted[0].topic.replace(iotclientId, "[AWS IoT thing name(replaced)]")
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

const consoleWidth = document.getElementById("console").clientWidth;
const consoleHeight = consoleWidth / 130 * 70;

let isBackgroundLayerUpdated = true;
const background_layer_sketch = function (p) {
    p.setup = function () {
        p.createCanvas(consoleWidth, consoleHeight, p.WEBGL);
        p.frameRate(5);
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
    p.preload = function () {
        // NOTE: p.loadImage 関数が Fetch API を使用しているため、
        // "img/TURTLEBOT3_BURGER.svg"を直接読み込もうとすると CORS 関連のエラーが発生した
        const svgString = '<svg width="785" height="784" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" overflow="hidden"><defs><clipPath id="clip0"><path d="M-15-14 770-14 770 770-15 770Z" fill-rule="evenodd" clip-rule="evenodd"/></clipPath></defs><g clip-path="url(#clip0)" transform="translate(15 14)"><path d="M249 627.194 314 627.194 314 444 444 444 444 627.194 509 627.194 379 756Z" fill="#00BFFF" fill-rule="evenodd"/><rect x="101" y="345" width="53" height="66" fill="#BFBFBF"/><rect x="598" y="345" width="53" height="66" fill="#BFBFBF"/><path d="M581 262.5C581 258.358 584.358 255 588.5 255L618.5 255C622.642 255 626 258.358 626 262.5L626 410.5C626 414.642 622.642 418 618.5 418L588.5 418C584.358 418 581 414.642 581 410.5Z" fill="#595959" fill-rule="evenodd"/><path d="M132 262.5C132 258.358 135.358 255 139.5 255L169.5 255C173.642 255 177 258.358 177 262.5L177 410.5C177 414.642 173.642 418 169.5 418L139.5 418C135.358 418 132 414.642 132 410.5Z" fill="#595959" fill-rule="evenodd"/><path d="M603.5 268C603.667 317.5 603.833 367 604 416.5L577.5 422 556 441.5 544 456.5 534 477 530 489 224.5 490 221 470 212 453 203.5 441 188.5 430.5 173.5 422.5 153.5 417.5 152 269.5 603.5 268Z" stroke="#595959" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" stroke-miterlimit="10" fill="#7F7F7F" fill-rule="evenodd"/><path d="M451.5 0C451.667 53.0675 451.833 106.135 452 159.203L425.5 165.099 404 186.004 392 202.086 382 224.063 378 236.928 72.5001 238 69 216.559 60 198.333 51.5 185.468 36.5 174.212 21.5001 165.635 1.50005 160.275 0 1.60805 451.5 0Z" stroke="#595959" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" stroke-miterlimit="10" fill="#7F7F7F" fill-rule="evenodd" transform="matrix(1 0 0 -1 151 274)"/><path d="M-0.565416 395.671C-10.3582 187.183 150.936 10.2216 359.694 0.416092 568.453-9.3894 745.624 151.674 755.417 360.162 765.209 568.65 603.916 745.612 395.157 755.417 186.398 765.223 9.22735 604.159-0.565416 395.671Z" stroke="#FF0000" stroke-width="26.6667" stroke-miterlimit="8" stroke-dasharray="80 26.6667" fill="none" fill-rule="evenodd"/><path d="M62 272.667C62 266.776 66.7757 262 72.6668 262L115.333 262C121.224 262 126 266.776 126 272.667L126 482.333C126 488.224 121.224 493 115.333 493L72.6668 493C66.7757 493 62 488.224 62 482.333Z" fill="#404040" fill-rule="evenodd"/><path d="M630 272.5C630 266.701 634.701 262 640.5 262L682.5 262C688.299 262 693 266.701 693 272.5L693 482.5C693 488.299 688.299 493 682.5 493L640.5 493C634.701 493 630 488.299 630 482.5Z" fill="#404040" fill-rule="evenodd"/><path d="M0 0 457.926 0.000104987" stroke="#595959" stroke-width="5" stroke-miterlimit="8" fill="none" fill-rule="evenodd" transform="matrix(-1 -8.74228e-08 -8.74228e-08 1 604.426 274.5)"/><path d="M430.198 148 466.79 153.633 487.432 181.8 494 298.222 463.975 410.889 428.321 463.467 404.864 480.367 380.469 486 355.136 478.489 334.494 464.406 302.593 425.911 282.889 372.394 268.815 316.061 266 234.378 274.444 182.739 293.21 159.267 326.988 148.939 430.198 148Z" fill="#AFABAB" fill-rule="evenodd"/><path d="M492.5 252C492.5 379.854 441.908 483.5 379.5 483.5 317.52 483.5 267.107 381.22 266.505 254.25" stroke="#595959" stroke-width="5" stroke-miterlimit="8" fill="none" fill-rule="evenodd"/><path d="M265.5 251.944C268.039 216.456 270.579 180.967 289.508 163.927 308.438 146.887 348.297 150.147 379.077 149.702 409.856 149.258 454.948 143.627 474.186 161.26 493.423 178.893 490.037 239.201 494.5 255.5" stroke="#595959" stroke-width="5" stroke-miterlimit="8" fill="none" fill-rule="evenodd"/><path d="M264.5 265C264.5 202.316 315.316 151.5 378 151.5 440.684 151.5 491.5 202.316 491.5 265 491.5 327.684 440.684 378.5 378 378.5 315.316 378.5 264.5 327.684 264.5 265Z" stroke="#595959" stroke-width="5" stroke-miterlimit="8" fill="#C9C9C9" fill-rule="evenodd"/><path d="M354.5 426.5C354.5 413.245 365.245 402.5 378.5 402.5 391.755 402.5 402.5 413.245 402.5 426.5 402.5 439.755 391.755 450.5 378.5 450.5 365.245 450.5 354.5 439.755 354.5 426.5Z" stroke="#595959" stroke-width="5" stroke-miterlimit="8" fill="#C9C9C9" fill-rule="evenodd"/><path d="M0 0 62.6024 98.5926" stroke="#595959" stroke-width="7" stroke-miterlimit="8" fill="none" fill-rule="evenodd" transform="matrix(-1 -8.74228e-08 -8.74228e-08 1 458.102 345.5)"/><path d="M297.5 345.5 361.085 444.093" stroke="#595959" stroke-width="7" stroke-miterlimit="8" fill="none" fill-rule="evenodd"/></g></svg>';
        p.turtleBot3Image = p.loadImage("data:image/svg+xml;base64," + btoa(svgString));
    };

    p.setup = function () {
        p.createCanvas(consoleWidth, consoleHeight, p.WEBGL);
        p.background(0, 0, 0, 0);
    };

    p.draw = function () {
        /*** 以下、初期化処理 ***/
        if (!isFrontLayerUpdated) {
            return;
        }
        isFrontLayerUpdated = false;
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

    p.push()
    p.translate(-consoleWidth / 2, -consoleHeight / 2);
    for (let row = 0; row < costmap.info.height; row++) {
        for (let col = 0; col < costmap.info.width; col++) {
            p.fill(p.lerpColor(colorFrom, colorTo, costmap.data[row][col] / 100));
            p.rect(col * cellSize, row * cellSize, cellSize, cellSize);
        }
    }
    p.pop();
}

function drawTurtleBot3(p, odom, costmap, cellSize, consoleWidth, consoleHeight) {
    const TURTLEBOT3_BURGER_REAL_SIZE_M = 0.21;
    const resolution = costmap.info.resolution;
    const trutlebot3Px = cellSize / resolution * TURTLEBOT3_BURGER_REAL_SIZE_M;

    const x = cellSize / resolution * (odom.pose.pose.position.x - costmap.info.origin.position.x);
    const y = cellSize / resolution * (odom.pose.pose.position.y - costmap.info.origin.position.y);

    p.push();
    p.imageMode(p5.CENTER);
    p.translate(-consoleWidth / 2, -consoleHeight / 2);
    p.image(p.turtleBot3Image, x, y, trutlebot3Px, trutlebot3Px);
    p.pop();
}


/* AWS IoT へ接続する */
setupAwsIot();

/* Sketch を DOM に追加 */
const backgroundLayerP5 = new p5(background_layer_sketch, "background-layer");
const frontLayerP5 = new p5(front_layer_sketch, "front-layer");