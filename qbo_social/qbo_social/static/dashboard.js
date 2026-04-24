/* QBO Social Dashboard - JavaScript */

let ws = null;
let reconnectTimeout = null;
let actionLogEntries = [];

// ========================================
// GESTION DES ONGLETS
// ========================================

function switchTab(tabName) {
    // Cacher tous les contenus
    const contents = document.querySelectorAll('.tab-content');
    contents.forEach(c => c.classList.remove('active'));

    // Désactiver tous les boutons
    const buttons = document.querySelectorAll('.tab-button');
    buttons.forEach(b => b.classList.remove('active'));

    // Activer l'onglet sélectionné
    document.getElementById(`tab-${tabName}`).classList.add('active');

    // Trouver le bouton qui correspond et l'activer
    buttons.forEach(b => {
        if (b.textContent.includes(tabName === 'dashboard' ? 'Dashboard' : 'Actions')) {
            b.classList.add('active');
        }
    });
}

// ========================================
// WEBSOCKET
// ========================================

function connect() {
    const wsUrl = `ws://${window.location.host}/ws`;
    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
        console.log('WebSocket connecté');
        document.getElementById('statusMsg').innerHTML = '<span class="status-ok">✓ Connecté</span>';
    };

    ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        updateDashboard(data);
    };

    ws.onerror = (error) => {
        console.error('Erreur WebSocket:', error);
        document.getElementById('statusMsg').innerHTML = '<span class="status-error">✗ Erreur de connexion</span>';
    };

    ws.onclose = () => {
        console.log('WebSocket déconnecté');
        document.getElementById('statusMsg').innerHTML = '<span class="status-warning">⚠ Déconnecté, reconnexion...</span>';

        // Reconnexion automatique après 3 secondes
        reconnectTimeout = setTimeout(connect, 3000);
    };
}

function sendCommand(command, params = {}) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        document.getElementById('statusMsg').innerHTML = '<span class="status-error">✗ Non connecté</span>';
        return;
    }

    ws.send(JSON.stringify({
        command: command,
        ...params
    }));
}
// ========================================
// ENVOI D'INTENTS (ACTIONS)
// ========================================

function sendIntent(intentType, payload = {}) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        alert('WebSocket non connecté !');
        return;
    }

    const command = {
        command: 'send_intent',
        intent_type: intentType,
        payload: payload
    };

    console.log('📤 Envoi intent:', intentType, payload);
    console.log('📤 Message WebSocket:', JSON.stringify(command));
    ws.send(JSON.stringify(command));

    // Logger l'action localement
    logAction(intentType, payload, 'sent');
}

function sendCustomSpeech() {
    const text = document.getElementById('customSpeechText').value.trim();
    if (!text) {
        alert('Veuillez entrer un texte');
        return;
    }
    sendIntent('SAY_SHORT_PHRASE', { text: text, language: 'fr' });
    document.getElementById('customSpeechText').value = '';
}

function sendCustomGreet() {
    const name = document.getElementById('customPersonName').value.trim();
    if (!name) {
        alert('Veuillez entrer un nom');
        return;
    }
    sendIntent('GREET_PERSON', { name: name });
    document.getElementById('customPersonName').value = '';
}

function logAction(intentType, payload, status) {
    const now = new Date();
    const time = now.toLocaleTimeString('fr-FR');

    const entry = {
        time: time,
        intent: intentType,
        payload: JSON.stringify(payload),
        status: status
    };

    actionLogEntries.unshift(entry);
    if (actionLogEntries.length > 20) {
        actionLogEntries.pop();
    }

    updateActionLog();
}

function updateActionLog() {
    const container = document.getElementById('actionLog');
    if (!container) return;

    if (actionLogEntries.length === 0) {
        container.innerHTML = '<div class="status-dim">Aucune action envoyée</div>';
        return;
    }

    let html = '';
    for (const entry of actionLogEntries) {
        const statusClass = entry.status === 'sent' ? 'action-log-success' : 'action-log-error';
        html += `
            <div class="action-log-entry">
                <span class="action-log-time">${entry.time}</span>
                <span class="action-log-intent">${entry.intent}</span>
                <span class="action-log-payload">${entry.payload}</span>
                <span class="${statusClass}">●</span>
            </div>
        `;
    }

    container.innerHTML = html;
}

// ========================================
// MISE À JOUR DASHBOARD
// ========================================
function updateDashboard(data) {
    // Timestamp
    const date = new Date(data.timestamp);
    document.getElementById('timestampDate').textContent = date.toLocaleDateString('fr-FR');
    document.getElementById('timestampTime').textContent = date.toLocaleTimeString('fr-FR');

    // Système
    const modeClass = data.mode === 'NORMAL' ? 'status-ok' :
                    (data.mode.includes('CRITICAL') || data.mode.includes('DEGRADED')) ? 'status-error' : 'status-warning';
    document.getElementById('mode').innerHTML = `<span class="${modeClass}">● ${data.mode}</span>`;
    document.getElementById('profile').textContent = data.profile;
    document.getElementById('missing').innerHTML = data.missing_nodes.length > 0 ?
        `<span class="status-error">${data.missing_nodes.join(', ')}</span>` :
        '<span class="status-ok">✓ Complet</span>';

    // Batterie
    document.getElementById('batteryV').textContent = data.battery.voltage.toFixed(2);
    updateBar('batteryBar', (data.battery.voltage - 11) / 3, true);
    document.getElementById('runtime').textContent = data.battery.runtime_min > 0 ?
        `${data.battery.runtime_min.toFixed(0)} min` : 'N/A';
    document.getElementById('charging').innerHTML = data.battery.charging ?
        '<span class="status-ok">En charge</span>' : '<span class="status-dim">Sur batterie</span>';

    // Thermique
    document.getElementById('cpuTemp').textContent = data.thermal.cpu_temp.toFixed(1);
    updateBar('cpuBar', data.thermal.cpu_temp / 80);
    document.getElementById('gpuTemp').textContent = data.thermal.gpu_temp.toFixed(1);
    updateBar('gpuBar', data.thermal.gpu_temp / 80);
    document.getElementById('ramPct').textContent = data.thermal.ram_pct.toFixed(1);
    updateBar('ramBar', data.thermal.ram_pct / 100);
    document.getElementById('power').textContent = data.thermal.power_w.toFixed(1);

    // Nœuds
    updateNodesTable(data.nodes);

    // World
    const faceStatus = data.world.face_stable ? 'STABLE' :
                     data.world.face_present ? 'DÉTECTÉ' : 'Aucun';
    const faceClass = data.world.face_stable ? 'status-ok' :
                    data.world.face_present ? 'status-warning' : 'status-dim';
    document.getElementById('face').innerHTML = `<span class="${faceClass}">${faceStatus}</span>`;
    document.getElementById('person').innerHTML = data.world.person_name ?
        `<span class="status-ok">${data.world.person_name}</span>` : '<span class="status-dim">---</span>';
    document.getElementById('engagement').textContent = data.world.engagement.toFixed(2);
    updateBar('engagementBar', data.world.engagement);
    document.getElementById('tracking').innerHTML = data.world.tracking_on ?
        '<span class="status-ok">ON</span>' : '<span class="status-dim">OFF</span>';
    document.getElementById('conversation').innerHTML = data.world.conv_active ?
        '<span class="status-ok">OUI</span>' : '<span class="status-dim">Non</span>';

    // Intent
    document.getElementById('rawIntent').textContent = data.intent.raw || '---';
    document.getElementById('filteredIntent').textContent = data.intent.filtered || '---';

    let statusText = '---';
    let statusClass = 'status-dim';
    if (data.intent.raw && data.intent.raw !== '---') {
        if (data.intent.raw === data.intent.filtered) {
            statusText = 'OK';
            statusClass = 'status-ok';
        } else if (!data.intent.filtered || data.intent.filtered === '---') {
            statusText = 'BLOQUÉ';
            statusClass = 'status-error';
        } else {
            statusText = 'DÉGRADÉ';
            statusClass = 'status-warning';
        }
    }
    document.getElementById('intentStatus').innerHTML = `<span class="${statusClass}">${statusText}</span>`;
    document.getElementById('intentReason').textContent = data.intent.reason || '--';

    // Compteurs
    document.getElementById('eventCount').textContent = data.counters.events;
    document.getElementById('intentCount').textContent = data.counters.intents;
    document.getElementById('blockedCount').textContent = data.counters.blocked;

    // Session
    updateSession(data.session, data.timestamp);

    // Position & Carte
    updatePosition(data.face_position, data.world.person_name);

    // Log
    updateLog(data.log);
}

function updateBar(elementId, ratio, isBattery = false) {
    const bar = document.getElementById(elementId);
    const percent = Math.max(0, Math.min(100, ratio * 100));
    bar.style.width = percent + '%';

    let color;
    if (isBattery) {
        color = percent < 20 ? '#ef4444' : percent < 40 ? '#eab308' : '#22c55e';
    } else {
        color = percent >= 85 ? '#ef4444' : percent >= 65 ? '#eab308' : '#22c55e';
    }
    bar.style.background = color;
}

function updateNodesTable(nodes) {
    const knownNodes = {
        "System": "qbo_arduqbo principal",
        "Qboard_1": "base + capteurs",
        "Qboard_3": "batterie",
        "Qboard_4": "IMU",
        "Qboard_5": "nez + bouche",
        "LCD": "ecran LCD",
        "qbo_dynamixel": "head_pan / head_tilt",
        "orin-nx-16g": "Orin NX (diag syst.)",
        "qbo_vision": "face_tracker"
    };

    let html = '';
    for (const [hw, desc] of Object.entries(knownNodes)) {
        const state = nodes[hw];
        let stateHtml, icon;
        if (state === true) {
            stateHtml = '<span class="status-ok">✓ OK</span>';
            icon = '✓';
        } else if (state === false) {
            stateHtml = '<span class="status-error">✗ ERROR</span>';
            icon = '✗';
        } else {
            stateHtml = '<span class="status-dim">? Attente</span>';
            icon = '?';
        }
        html += `<tr>
            <td>${icon} ${hw}</td>
            <td>${stateHtml}</td>
            <td class="status-dim">${desc}</td>
        </tr>`;
    }

    // Nœuds découverts dynamiquement
    for (const [hw, state] of Object.entries(nodes)) {
        if (!knownNodes[hw]) {
            const stateHtml = state ? '<span class="status-ok">✓ OK</span>' : '<span class="status-error">✗ ERROR</span>';
            const icon = state ? '✓' : '✗';
            html += `<tr>
                <td>${icon} ${hw}</td>
                <td>${stateHtml}</td>
                <td class="status-dim">découvert dynamiquement</td>
            </tr>`;
        }
    }

    document.getElementById('nodesTable').innerHTML = html;
}

function updateLog(logEntries) {
    const container = document.getElementById('logContainer');

    if (logEntries.length === 0) {
        container.innerHTML = '<div class="status-dim">Aucun événement</div>';
        return;
    }

    let html = '';
    for (const entry of logEntries) {
        let typeClass = 'log-type-event';
        let content = '';

        if (entry.type === 'event') {
            typeClass = 'log-type-event';
            content = `<span class="${typeClass}">${entry.event_type}</span>`;
            if (entry.source) content += ` <span class="status-dim">[${entry.source}]</span>`;

            // Afficher les données spécifiques aux événements visuels
            if (entry.data && Object.keys(entry.data).length > 0) {
                const dataStr = Object.entries(entry.data)
                    .filter(([k, v]) => v && k !== 'payload')
                    .map(([k, v]) => `${k}=${v}`)
                    .join(', ');
                if (dataStr) {
                    content += ` <span class="status-dim">(${dataStr})</span>`;
                }
            }
        } else if (entry.type === 'intent') {
            typeClass = 'log-type-intent';
            content = `<span class="${typeClass}">INTENT ${entry.intent_type}</span>`;
            if (entry.reason) content += ` <span class="status-dim">${entry.reason}</span>`;
        } else if (entry.type === 'trace') {
            typeClass = 'log-type-trace';
            const status = entry.suppressed ? '<span class="status-error">[COOLDOWN]</span>' : '<span class="status-ok">[OK]</span>';
            content = `<span class="${typeClass}">TRACE</span> ${entry.chosen_intent} ${status}`;

            // Afficher la raison détaillée de la trace
            if (entry.reason) {
                content += `<br><span class="status-dim" style="margin-left: 70px; font-size: 0.85em;">${entry.reason}</span>`;
            }
        }

        html += `<div class="log-entry">
            <span class="log-time">${entry.time}</span>${content}
        </div>`;
    }

    container.innerHTML = html;
    container.scrollTop = container.scrollHeight;
}

function updateSession(session, currentTimestamp) {
    const now = new Date(currentTimestamp).getTime() / 1000;
    const SESSION_TIMEOUT = 10.0; // Correspondant au behavior_engine

    if (!session.active) {
        // Aucune session active
        document.getElementById('sessionStatus').innerHTML = '<span class="status-dim">● Aucune session</span>';
        document.getElementById('sessionPerson').textContent = '--';
        document.getElementById('sessionDuration').textContent = '--';
        document.getElementById('sessionLastSeen').textContent = '--';
        document.getElementById('sessionTimeout').textContent = '--';
        return;
    }

    // Session active
    const person = session.person_name || '?';
    const duration = session.started_at ? Math.floor(now - session.started_at) : 0;
    const lastSeenAgo = session.last_recognition ? Math.floor(now - session.last_recognition) : null;
    const faceLostAgo = session.face_lost_at ? Math.floor(now - session.face_lost_at) : null;

    // État de la session
    let statusHtml = '';
    if (session.status === 'greeted') {
        statusHtml = '<span class="status-ok">● Active</span>';
    } else if (session.status === 'lost') {
        const timeoutRemaining = SESSION_TIMEOUT - faceLostAgo;
        if (timeoutRemaining > 0) {
            statusHtml = `<span class="status-warning">● Visage perdu (${timeoutRemaining.toFixed(0)}s)</span>`;
        } else {
            statusHtml = '<span class="status-error">● Timeout imminent</span>';
        }
    } else {
        statusHtml = '<span class="status-dim">● Indéfini</span>';
    }

    document.getElementById('sessionStatus').innerHTML = statusHtml;
    document.getElementById('sessionPerson').innerHTML = `<span class="status-ok">${person}</span>`;
    document.getElementById('sessionDuration').textContent = `${duration}s`;

    if (lastSeenAgo !== null) {
        document.getElementById('sessionLastSeen').textContent = lastSeenAgo === 0 ? 
            'maintenant' : `il y a ${lastSeenAgo}s`;
    } else {
        document.getElementById('sessionLastSeen').textContent = '--';
    }

    if (session.status === 'lost' && faceLostAgo !== null) {
        const timeoutRemaining = Math.max(0, SESSION_TIMEOUT - faceLostAgo);
        const timeoutClass = timeoutRemaining < 3 ? 'status-error' : 
                           timeoutRemaining < 6 ? 'status-warning' : 'status-ok';
        document.getElementById('sessionTimeout').innerHTML = 
            `<span class="${timeoutClass}">${timeoutRemaining.toFixed(1)}s</span>`;
    } else {
        document.getElementById('sessionTimeout').textContent = '--';
    }
}

function updatePosition(position, personName) {
    // Mise à jour des données textuelles
    if (!position.timestamp) {
        // Pas de visage détecté
        document.getElementById('posXYZ').textContent = '--';
        document.getElementById('posDist').textContent = '--';
        document.getElementById('posAngle').textContent = '--';
        drawMap(null, null);
        return;
    }

    const x = position.x;
    const y = position.y;
    const z = position.z;
    const dist = position.distance;

    // Afficher coordonnées
    document.getElementById('posXYZ').textContent = `(${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)})m`;
    document.getElementById('posDist').textContent = `${dist.toFixed(2)}m`;

    // Calculer angle (atan2 pour orientation)
    const angleDeg = (Math.atan2(y, x) * 180 / Math.PI).toFixed(1);
    document.getElementById('posAngle').textContent = `${angleDeg}°`;

    // Dessiner la carte 2D
    drawMap(position, personName || 'Personne');
}

function drawMap(position, personName) {
    const canvas = document.getElementById('mapCanvas');
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;

    // Effacer canvas
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = '#0f172a';
    ctx.fillRect(0, 0, width, height);

    const centerX = width / 2;
    const centerY = height / 2;

    // Échelle : adapter selon la distance (min 1m, max 3m visible)
    const maxDist = position ? Math.max(2.0, position.distance + 0.5) : 2.0;
    const scale = (width * 0.4) / maxDist; // pixels par mètre

    // Dessiner grille
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 1;
    for (let i = -3; i <= 3; i++) {
        if (i === 0) continue;
        const offset = i * scale;
        // Lignes verticales
        ctx.beginPath();
        ctx.moveTo(centerX + offset, 0);
        ctx.lineTo(centerX + offset, height);
        ctx.stroke();
        // Lignes horizontales
        ctx.beginPath();
        ctx.moveTo(0, centerY + offset);
        ctx.lineTo(width, centerY + offset);
        ctx.stroke();
    }

    // Axes principaux
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 2;
    // Axe X (rouge)
    ctx.strokeStyle = '#ef4444';
    ctx.beginPath();
    ctx.moveTo(0, centerY);
    ctx.lineTo(width, centerY);
    ctx.stroke();
    // Axe Y (vert)
    ctx.strokeStyle = '#22c55e';
    ctx.beginPath();
    ctx.moveTo(centerX, 0);
    ctx.lineTo(centerX, height);
    ctx.stroke();

    // Légende des axes
    ctx.fillStyle = '#64748b';
    ctx.font = '10px monospace';
    ctx.fillText('Y →', width - 25, centerY - 5);
    ctx.fillText('X ↑', centerX + 5, 15);

    // Robot au centre (triangle pointant vers le haut = axe X)
    ctx.fillStyle = '#3b82f6';
    ctx.beginPath();
    ctx.moveTo(centerX, centerY - 15);     // Sommet
    ctx.lineTo(centerX - 10, centerY + 10); // Bas gauche
    ctx.lineTo(centerX + 10, centerY + 10); // Bas droit
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = '#60a5fa';
    ctx.lineWidth = 2;
    ctx.stroke();

    // Label robot
    ctx.fillStyle = '#60a5fa';
    ctx.font = 'bold 11px monospace';
    ctx.fillText('Qbo', centerX - 25, centerY + 25);

    // Si position valide, dessiner la personne
    if (position && position.timestamp) {
        const px = centerX + (position.y * scale);  // Y robot = axe horizontal (latéral)
        const py = centerY - (position.x * scale);  // X robot = axe vertical (avant/arrière, inversé canvas)

        // Ligne de distance
        ctx.strokeStyle = '#eab308';
        ctx.lineWidth = 1;
        ctx.setLineDash([5, 5]);
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.lineTo(px, py);
        ctx.stroke();
        ctx.setLineDash([]);

        // Distance au milieu de la ligne
        const midX = (centerX + px) / 2;
        const midY = (centerY + py) / 2;
        ctx.fillStyle = '#eab308';
        ctx.font = '10px monospace';
        ctx.fillText(`${position.distance.toFixed(2)}m`, midX + 5, midY - 5);

        // Cercle pour la personne
        ctx.fillStyle = '#22c55e';
        ctx.beginPath();
        ctx.arc(px, py, 8, 0, 2 * Math.PI);
        ctx.fill();
        ctx.strokeStyle = '#4ade80';
        ctx.lineWidth = 2;
        ctx.stroke();

        // Nom de la personne
        ctx.fillStyle = '#4ade80';
        ctx.font = 'bold 11px monospace';
        const nameWidth = ctx.measureText(personName).width;
        ctx.fillText(personName, px - nameWidth / 2, py - 15);

        // Coordonnées (X, Y selon le robot)
        ctx.fillStyle = '#64748b';
        ctx.font = '9px monospace';
        const coordText = `(${position.x.toFixed(1)}, ${position.y.toFixed(1)})`;
        const coordWidth = ctx.measureText(coordText).width;
        ctx.fillText(coordText, px - coordWidth / 2, py + 20);
    }
}

// Connexion initiale au chargement de la page
window.addEventListener('DOMContentLoaded', () => {
    console.log('DOM chargé, initialisation...');

    // Vérifier que les onglets existent
    const tabDashboard = document.getElementById('tab-dashboard');
    const tabActions = document.getElementById('tab-actions');

    console.log('tab-dashboard:', tabDashboard);
    console.log('tab-actions:', tabActions);

    if (tabDashboard && tabActions) {
        console.log('✓ Onglets trouvés');
        // S'assurer que seul dashboard est actif au départ
        tabDashboard.classList.add('active');
        tabActions.classList.remove('active');
    } else {
        console.error('✗ Onglets manquants !');
    }

    // Connexion WebSocket
    connect();
});