const canvas = document.getElementById("canvas");  // canvas要素を取得
const ctx = canvas.getContext("2d");  // コンテキストを取得

function start() {
    // canvasのスタイルのサイズを取得
    var canvas_width = canvas.clientWidth;
    var canvas_height = canvas.clientHeight;

    // canvasコンテキスト(?)のサイズとスタイルのサイズを統一
    // こうしないと、図形描画時の縦横比がおかしくなる
    canvas.width = canvas_width;
    canvas.height = canvas_height;
}

var i = 0;
function draw() {
    var canvas_width = canvas.clientWidth;
    var canvas_height = canvas.clientHeight;
    var color = "#eaf4fc";

    background(color);

    ctx.rect(10 + i, 10 + i, 100 + i, 100 + i);
    ctx.stroke();
    i += 2;
    i = i % Math.floor(canvas_width / 2);
}

/**
 * canvasをリセットし、背景色を設定する
 * @param {背景に設定する色} color 
 */
function background(color) {
    var canvas_width = canvas.clientWidth;
    var canvas_height = canvas.clientHeight;

    ctx.beginPath();
    ctx.fillStyle = color;
    ctx.fillRect(0, 0, canvas_width, canvas_height);
}


/**** Processingぽくするための処理 ****/
const fps = 30;
start();

/**
 * FPSなどを設定し、draw関数を呼び出す
 */
var intervalId = window.setInterval(draw, 1000 / fps);