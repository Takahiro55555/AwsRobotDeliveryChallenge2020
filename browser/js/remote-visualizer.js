const awsIot = require('aws-iot-device-sdk');
const gm_subscribe_topic = iotclientId + "/ros_to_remote_console/obstacle_detector/merged_costmap/trimed";

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

    deviceIot.on('message', function (_topic, payload) {
        const json = JSON.parse(payload.toString());
        console.log(json);
        isUpdated = true;
    });

    deviceIot.subscribe(gm_subscribe_topic, undefined, function (err, granted) {
        if (err) {
            console.log('subscribe error: ' + err);
        } else {
            console.log('subscribe success');
        }
    });
}

setupAwsIot();


const static_layer_sketch = function (p) {
    p.setup = function () {
        p.createCanvas(700, 700);
        p.background(200);
    };

    let counter = 0;
    p.draw = function () {
        p.background(200);
        p.ellipse(counter, counter, 100, 100);
        counter++;
        counter = counter % 700;
    };
};

const dynmic_layer_sketch = function (p) {
    p.setup = function () {
        p.createCanvas(700, 700);
        p.background(0, 0, 0, 0);
    };

    let counter = 0;
    p.draw = function () {
        p.setup();
        p.ellipse(counter, 700 - counter, 100, 100);
        counter++;
        counter = counter % 700;
    };
};

new p5(static_layer_sketch, "static-layer");
new p5(dynmic_layer_sketch, "dynamic-layer");