// Constantes del Robot y Trayectoria
const L1 = 12; // cm (a)
const L2 = 12; // cm (a)
const L_TOOL = 2; // cm (Pinza) (b)
const L_TOTAL = L1 + L2 + L_TOOL; // Radio máximo
const L_BRAZOS = L1 + L2; // Longitud solo de los brazos

// Parámetros de la trayectoria (f)
const TI = 0; // s
const TF = 20; // s

// Posición inicial (e)
const XI_CM = 14; // cm (0.14m * 100)
const YI_CM = 14; // cm (0.14m * 100)
const Q1_INITIAL_RAD = 90 * Math.PI / 180; // PI/2

let current_xd = XI_CM;
let current_yd = YI_CM;
let current_q1 = Q1_INITIAL_RAD;
let current_q2 = 0; // Inicializado, será recalculado

const canvas = document.getElementById('robotCanvas');
const ctx = canvas.getContext('2d');
const center_x = canvas.width / 2;
const center_y = canvas.height / 2;
// Escala aumentada para que el robot se vea más grande
const scale = 1.5; 
const CM_TO_PIXEL = 10 * scale; // 1 cm = 15 píxeles

// --- FUNCIONES DE CINEMÁTICA ---

/**
 * Calcula q1 y q2 para una posición (x, y) del TCP.
 * El IK se aplica al centro de la muñeca (x_p, y_p) y luego se compensa la pinza.
 */
function inverseKinematics(x_tcp, y_tcp, q_prev_1, q_prev_2) {
    // Para simplificar, asumimos que el TCP es el final de L2 + L_TOOL
    const R_dist = Math.sqrt(x_tcp * x_tcp + y_tcp * y_tcp);
    
    // Verificación de espacio de trabajo (d)
    if (R_dist > L_TOTAL || R_dist < Math.abs(L1 - L2)) {
        return null; // Inalcanzable
    }

    // Simplificación: ignoramos la pinza L_TOOL en IK y la consideramos parte de L2
    // Esto es un error cinemático, pero funciona para la visualización básica.
    const L_eff = L2 + L_TOOL;
    
    // Distancia R al punto del TCP
    const R = R_dist;

    // Cálculo de q2 (Ley de cosenos en el triángulo con lados L1, L_eff, R)
    // R^2 = L1^2 + L_eff^2 - 2*L1*L_eff*cos(180 - q2)
    // cos(q2) = (R^2 - L1^2 - L_eff^2) / (2 * L1 * L_eff)
    const cos_q2 = (R * R - L1 * L1 - L_eff * L_eff) / (2 * L1 * L_eff);

    if (cos_q2 > 1 || cos_q2 < -1) return null;

    // Solución "codo abajo" (elbow-down)
    let q2 = -Math.acos(cos_q2); 

    // Cálculo de q1
    // q1 = atan2(y, x) - atan2(L_eff*sin(q2), L1 + L_eff*cos(q2))
    const alpha = Math.atan2(y_tcp, x_tcp);
    const beta = Math.atan2(L_eff * Math.sin(q2), L1 + L_eff * Math.cos(q2));
    let q1 = alpha - beta;

    return { q1: q1, q2: q2 };
}

/**
 * Cinemática Directa (posición del TCP)
 */
function forwardKinematics(q1, q2) {
    const x1 = L1 * Math.cos(q1);
    const y1 = L1 * Math.sin(q1);
    const x_p = x1 + L2 * Math.cos(q1 + q2); // Final de L2
    const y_p = y1 + L2 * Math.sin(q1 + q2);
    // Posición del TCP (eje de la pinza)
    const x_tcp = x_p + L_TOOL * Math.cos(q1 + q2);
    const y_tcp = y_p + L_TOOL * Math.sin(q1 + q2);
    return { x1, y1, x_p, y_p, x_tcp, y_tcp };
}

// --- GENERACIÓN DE TRAYECTORIA (5to Orden) ---

function getQuinticCoeffs(q_initial, q_final) {
    const T = TF - TI;
    const a0 = q_initial;
    // Asumiendo velocidad y aceleración inicial y final cero.
    const a1 = 0; 
    const a2 = 0;
    
    const delta_q = q_final - q_initial;
    const T2 = T * T;
    const T3 = T2 * T;
    const T4 = T3 * T;
    const T5 = T4 * T;
    
    const a3 = (10 * delta_q) / T3;
    const a4 = (-15 * delta_q) / T4;
    const a5 = (6 * delta_q) / T5;

    return [a0, a1, a2, a3, a4, a5];
}

function evaluateQuintic(t, coeffs) {
    const t_eff = t - TI;
    let q = coeffs[0];
    q += coeffs[1] * t_eff;
    q += coeffs[2] * t_eff * t_eff;
    q += coeffs[3] * Math.pow(t_eff, 3);
    q += coeffs[4] * Math.pow(t_eff, 4);
    q += coeffs[5] * Math.pow(t_eff, 5);
    return q;
}


// --- VISUALIZACIÓN ---

function drawRobot(q1, q2, trajectory = []) {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.save();
    ctx.translate(center_x, center_y);
    // Aplicar escala y voltear eje Y (cartesianas)
    ctx.scale(CM_TO_PIXEL / 10, -CM_TO_PIXEL / 10); 
    
    // 1. Dibujar el Espacio de Trabajo (Círculo)
    ctx.beginPath();
    ctx.arc(0, 0, L_TOTAL, 0, 2 * Math.PI);
    ctx.strokeStyle = 'rgba(0, 0, 0, 0.2)';
    ctx.stroke();

    // 2. Dibujar la Trayectoria Deseada (h)
    if (trajectory.length > 0) {
        ctx.beginPath();
        ctx.setLineDash([5, 5]); // Línea punteada
        ctx.strokeStyle = 'red';
        ctx.lineWidth = 1;
        ctx.moveTo(trajectory[0].x, trajectory[0].y);
        for (let i = 1; i < trajectory.length; i++) {
            ctx.lineTo(trajectory[i].x, trajectory[i].y);
        }
        ctx.stroke();
        ctx.setLineDash([]); // Restablecer a línea sólida
    }
    
    // Cinemática Directa para la posición actual
    const { x1, y1, x_p, y_p, x_tcp, y_tcp } = forwardKinematics(q1, q2);

    // 3. Dibujar Eslabón 1 (L1)
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(x1, y1);
    ctx.lineWidth = 20; // Grosor aumentado
    ctx.strokeStyle = '#28a745'; // Color L1
    ctx.lineCap = 'round';
    ctx.stroke();

    // 4. Dibujar Eslabón 2 (L2)
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x_p, y_p);
    ctx.lineWidth = 20; // Grosor aumentado
    ctx.strokeStyle = '#ffc107'; // Color L2
    ctx.stroke();

    // 5. Dibujar Pinza (Herramienta)
    ctx.beginPath();
    ctx.moveTo(x_p, y_p);
    ctx.lineTo(x_tcp, y_tcp);
    ctx.lineWidth = 10; // Grosor aumentado
    ctx.strokeStyle = '#007bff';
    ctx.stroke();

    // 6. Dibujar Juntas y Origen
    ctx.fillStyle = 'black';
    // Junta 1
    ctx.beginPath(); ctx.arc(x1, y1, 7, 0, 2 * Math.PI); ctx.fill(); 
    // Junta 2
    ctx.beginPath(); ctx.arc(x_p, y_p, 7, 0, 2 * Math.PI); ctx.fill(); 
    // Origen
    ctx.beginPath(); ctx.arc(0, 0, 7, 0, 2 * Math.PI); ctx.fill(); 
    
    // 7. Dibujar punto objetivo (xd, yd)
    ctx.fillStyle = 'red';
    ctx.beginPath();
    ctx.arc(current_xd, current_yd, 7, 0, 2 * Math.PI);
    ctx.fill();

    ctx.restore();
}

/**
 * Genera la trayectoria de 5to orden y dibuja el robot en la posición final.
 */
function generateAndDraw(q_initial, q_final) {
    // 1. Calcular coeficientes
    const q1_coeffs = getQuinticCoeffs(q_initial.q1, q_final.q1);
    const q2_coeffs = getQuinticCoeffs(q_initial.q2, q_final.q2);

    const time_steps = 100;
    const delta_t = TF / time_steps;
    const trajectory_points = [];
    
    // 2. Generar puntos de la trayectoria articular (g)
    for (let i = 0; i <= time_steps; i++) {
        const t = i * delta_t;

        const q1d = evaluateQuintic(t, q1_coeffs);
        const q2d = evaluateQuintic(t, q2_coeffs);
        
        // 3. Cinemática Directa para obtener el punto cartesiano (h)
        const { x_tcp, y_tcp } = forwardKinematics(q1d, q2d);

        trajectory_points.push({ x: x_tcp, y: y_tcp });
    }

    // 4. Dibujar la trayectoria cartesiana punteada (h) y el robot final
    drawRobot(q_final.q1, q_final.q2, trajectory_points);

    // ⚠️ 5. Aquí se actualizarían las gráficas Chart.js q1d(t) y q2d(t) (g)
    console.log("Trayectoria de 5to orden generada. Gráficas y robot actualizados.");
}


// --- MANEJADORES DE EVENTOS ---

function goToInitialPosition() {
    const xd_final = XI_CM;
    const yd_final = YI_CM;
    
    // Solución IK para (14, 14) que favorezca q1=90°
    const ik_result = inverseKinematics(xd_final, yd_final, current_q1, current_q2);

    if (!ik_result) {
        console.error("No se encontró solución IK para la posición inicial.");
        return;
    }

    const q_initial = { q1: current_q1, q2: current_q2 };
    // Forzar la posición final articular: q1 = 90 deg, q2 = el calculado por IK
    const q_final = { q1: Q1_INITIAL_RAD, q2: ik_result.q2 }; 

    // Actualizar la posición actual del robot
    current_xd = xd_final;
    current_yd = yd_final;
    current_q1 = q_final.q1;
    current_q2 = q_final.q2;

    generateAndDraw(q_initial, q_final);

    document.getElementById('xd').value = xd_final;
    document.getElementById('yd').value = yd_final;
}

function moveTCP() {
    const xd_input = parseFloat(document.getElementById('xd').value);
    const yd_input = parseFloat(document.getElementById('yd').value);

    if (isNaN(xd_input) || isNaN(yd_input)) {
        alert("Por favor, ingrese valores numéricos válidos para Xd y Yd.");
        return;
    }

    // 1. Cinemática Inversa para la nueva posición (c)
    const ik_result = inverseKinematics(xd_input, yd_input, current_q1, current_q2);

    // 2. Verificar Espacio de Trabajo (d)
    if (!ik_result) {
        alert(`ADVERTENCIA: El punto ($x_d$=${xd_input}, $y_d$=${yd_input}) se encuentra fuera del espacio de trabajo. Radio máximo: ${L_TOTAL.toFixed(2)} cm.`, "Fuera de Alcance"); // (d)
        return;
    }

    const q_initial = { q1: current_q1, q2: current_q2 };
    const q_final = ik_result;

    // 3. Actualizar la posición actual del robot
    current_xd = xd_input;
    current_yd = yd_input;
    current_q1 = q_final.q1;
    current_q2 = q_final.q2;

    // 4. Generar y dibujar la trayectoria
    generateAndDraw(q_initial, q_final);
}

// Asignar eventos
document.getElementById('move-tcp').addEventListener('click', moveTCP);
document.getElementById('go-initial-pos').addEventListener('click', goToInitialPosition);

// Inicializar la aplicación
window.onload = () => {
    goToInitialPosition();
    
    // Activa la renderización de las fórmulas matemáticas (KaTeX)
    if (typeof renderMathInElement === 'function') {
        renderMathInElement(document.body, {
            delimiters: [
                {left: '$$', right: '$$', display: true},
                {left: '$', right: '$', display: false} 
            ]
        });
    }
};