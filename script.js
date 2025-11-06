const canvas = document.getElementById("robotCanvas");
const ctx = canvas.getContext("2d");

const L1 = 12;
const L2 = 12;
const pinza = 2;

let xd = 14;
let yd = 14;
let q1 = Math.PI / 2;
let q2 = Math.PI / 2;

// ===================== FUNCIONES =====================

// Cinemática inversa (devuelve [q1, q2])
function cinematicaInversa(x, y) {
    const D = (x ** 2 + y ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2);
    if (Math.abs(D) > 1) return null;
    const q2 = Math.atan2(Math.sqrt(1 - D ** 2), D);
    const q1 = Math.atan2(y, x) - Math.atan2(L2 * Math.sin(q2), L1 + L2 * Math.cos(q2));
    return [q1, q2];
}

// Cinemática directa
function cinematicaDirecta(q1, q2) {
    const x = L1 * Math.cos(q1) + L2 * Math.cos(q1 + q2);
    const y = L1 * Math.sin(q1) + L2 * Math.sin(q1 + q2);
    return [x, y];
}

// Dibuja cuadrícula cartesiana
function dibujarEjes() {
    const step = 20;
    ctx.save();
    ctx.strokeStyle = "#ddd";
    ctx.lineWidth = 1;

    for (let x = 0; x <= canvas.width; x += step) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, canvas.height);
        ctx.stroke();
    }

    for (let y = 0; y <= canvas.height; y += step) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(canvas.width, y);
        ctx.stroke();
    }

    // Ejes principales
    ctx.strokeStyle = "#000";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, canvas.height / 2);
    ctx.lineTo(canvas.width, canvas.height / 2);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(canvas.width / 2, 0);
    ctx.lineTo(canvas.width / 2, canvas.height);
    ctx.stroke();

    ctx.restore();
}

// Dibuja el robot en el canvas
function dibujarRobot() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    dibujarEjes();

    ctx.save();
    ctx.translate(canvas.width / 2, canvas.height / 2);
    ctx.scale(20, -20); // Escala: 20 px = 1 cm aprox

    const [x1, y1] = [L1 * Math.cos(q1), L1 * Math.sin(q1)];
    const [x2, y2] = cinematicaDirecta(q1, q2);
    const [xh, yh] = [
        x2 + pinza * Math.cos(q1 + q2),
        y2 + pinza * Math.sin(q1 + q2)
    ];

    // Brazos
    ctx.lineWidth = 0.3;
    ctx.strokeStyle = "red";
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(x1, y1);
    ctx.stroke();

    ctx.strokeStyle = "green";
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();

    // Herramienta (pinza)
    ctx.strokeStyle = "orange";
    ctx.beginPath();
    ctx.moveTo(x2, y2);
    ctx.lineTo(xh, yh);
    ctx.stroke();

    // Juntas
    ctx.fillStyle = "black";
    ctx.beginPath();
    ctx.arc(0, 0, 0.4, 0, 2 * Math.PI);
    ctx.fill();

    ctx.beginPath();
    ctx.arc(x1, y1, 0.4, 0, 2 * Math.PI);
    ctx.fill();

    ctx.beginPath();
    ctx.arc(x2, y2, 0.4, 0, 2 * Math.PI);
    ctx.fill();

    ctx.restore();
}

// ===================== EVENTOS =====================
document.getElementById("move-tcp").addEventListener("click", () => {
    const newXd = parseFloat(document.getElementById("xd").value);
    const newYd = parseFloat(document.getElementById("yd").value);

    const q = cinematicaInversa(newXd, newYd);
    if (!q) {
        alert("El punto está fuera del espacio de trabajo.");
        return;
    }
    [q1, q2] = q;
    dibujarRobot();
});

document.getElementById("go-initial-pos").addEventListener("click", () => {
    q1 = Math.PI / 2;
    q2 = Math.PI / 2;
    xd = 14;
    yd = 14;
    document.getElementById("xd").value = xd;
    document.getElementById("yd").value = yd;
    dibujarRobot();
});

// ===================== INICIO =====================
dibujarRobot();
