const DEFAULT_CENTER = [42.641314, 77.073812];

const map = L.map('map', {
  zoomControl: true,
  preferCanvas: true
}).setView(DEFAULT_CENTER, 16);

L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 20,
  attribution: '&copy; OpenStreetMap contributors'
}).addTo(map);

const controlPoints = [];
let routeLine = L.polyline([], {
  color: '#4aa3ff',
  weight: 4,
  opacity: 0.95
}).addTo(map);

let generatedLine = L.polyline([], {
  color: '#31d0aa',
  weight: 2,
  opacity: 0.55,
  dashArray: '5 8'
}).addTo(map);

const els = {
  missionName: document.getElementById('missionName'),
  speed: document.getElementById('speed'),
  arrivalRadius: document.getElementById('arrivalRadius'),
  spacing: document.getElementById('spacing'),
  controlCount: document.getElementById('controlCount'),
  generatedCount: document.getElementById('generatedCount'),
  routeLength: document.getElementById('routeLength'),
  estimatedTime: document.getElementById('estimatedTime'),
  validation: document.getElementById('validation'),
  statusText: document.getElementById('statusText'),
  importInput: document.getElementById('importInput'),
  toast: document.getElementById('toast')
};

function showToast(message) {
  els.toast.textContent = message;
  els.toast.classList.add('show');
  setTimeout(() => els.toast.classList.remove('show'), 1800);
}

function markerColor(index) {
  if (index === 0) return '#31d0aa';
  if (index === controlPoints.length - 1) return '#ff5d6c';
  return '#4aa3ff';
}

function makeIcon(index) {
  const color = markerColor(index);
  const html = `
    <div style="
      width:28px;height:28px;border-radius:50%;
      background:${color};border:3px solid white;
      display:grid;place-items:center;color:#06111d;
      font-weight:800;font-size:11px;
      box-shadow:0 5px 18px rgba(0,0,0,.35)">
      ${index + 1}
    </div>`;
  return L.divIcon({
    html,
    className: '',
    iconSize: [28, 28],
    iconAnchor: [14, 14]
  });
}

function refreshMarkers() {
  controlPoints.forEach((item, index) => {
    item.marker.setIcon(makeIcon(index));
    item.marker.unbindTooltip();
    const label = index === 0
      ? `START · ${index + 1}`
      : index === controlPoints.length - 1
        ? `FINISH · ${index + 1}`
        : `WP ${index + 1}`;
    item.marker.bindTooltip(label, {
      permanent: false,
      direction: 'top',
      offset: [0, -12],
      className: 'marker-label'
    });
  });
}

function addControlPoint(latlng) {
  const marker = L.marker(latlng, { draggable: true }).addTo(map);
  const item = { marker };

  marker.on('drag', updateMission);
  marker.on('dragend', updateMission);
  marker.on('contextmenu', () => {
    const index = controlPoints.indexOf(item);
    if (index >= 0) {
      map.removeLayer(item.marker);
      controlPoints.splice(index, 1);
      updateMission();
    }
  });

  controlPoints.push(item);
  updateMission();
}

map.on('click', e => addControlPoint(e.latlng));

function toRad(value) {
  return value * Math.PI / 180;
}

function distanceMeters(a, b) {
  const R = 6371000;
  const dLat = toRad(b.lat - a.lat);
  const dLng = toRad(b.lng - a.lng);
  const lat1 = toRad(a.lat);
  const lat2 = toRad(b.lat);

  const h = Math.sin(dLat / 2) ** 2 +
            Math.cos(lat1) * Math.cos(lat2) *
            Math.sin(dLng / 2) ** 2;

  return 2 * R * Math.atan2(Math.sqrt(h), Math.sqrt(1 - h));
}

function interpolate(a, b, fraction) {
  return {
    lat: a.lat + (b.lat - a.lat) * fraction,
    lng: a.lng + (b.lng - a.lng) * fraction
  };
}

function getControlLatLngs() {
  return controlPoints.map(item => item.marker.getLatLng());
}

function generateWaypoints(points, spacingMeters) {
  if (points.length === 0) return [];
  if (points.length === 1) return [{ lat: points[0].lat, lng: points[0].lng }];

  const output = [{ lat: points[0].lat, lng: points[0].lng }];

  for (let i = 0; i < points.length - 1; i++) {
    const a = points[i];
    const b = points[i + 1];
    const segmentDistance = distanceMeters(a, b);
    const pieces = Math.max(1, Math.ceil(segmentDistance / spacingMeters));

    for (let step = 1; step <= pieces; step++) {
      const p = interpolate(a, b, step / pieces);
      output.push({ lat: p.lat, lng: p.lng });
    }
  }

  return output;
}

function totalDistance(points) {
  let total = 0;
  for (let i = 0; i < points.length - 1; i++) {
    total += distanceMeters(points[i], points[i + 1]);
  }
  return total;
}

function formatDistance(meters) {
  if (meters < 1000) return `${meters.toFixed(1)} м`;
  return `${(meters / 1000).toFixed(2)} км`;
}

function formatTime(seconds) {
  if (!Number.isFinite(seconds) || seconds <= 0) return '—';
  const minutes = Math.round(seconds / 60);
  if (minutes < 60) return `≈ ${minutes} мин`;
  const h = Math.floor(minutes / 60);
  const m = minutes % 60;
  return `≈ ${h} ч ${m} мин`;
}

function buildMission() {
  const control = getControlLatLngs();
  const spacing = Math.max(2, Number(els.spacing.value) || 8);
  const generated = generateWaypoints(control, spacing);

  return {
    schemaVersion: 1,
    missionName: els.missionName.value.trim() || 'Untitled Mission',
    createdAt: new Date().toISOString(),
    speedPercent: Math.max(5, Math.min(100, Number(els.speed.value) || 35)),
    arrivalRadiusMeters: Math.max(1, Number(els.arrivalRadius.value) || 3),
    waypointSpacingMeters: spacing,
    home: control.length ? {
      lat: Number(control[0].lat.toFixed(7)),
      lng: Number(control[0].lng.toFixed(7))
    } : null,
    controlPoints: control.map((p, index) => ({
      id: index + 1,
      lat: Number(p.lat.toFixed(7)),
      lng: Number(p.lng.toFixed(7))
    })),
    waypoints: generated.map((p, index) => ({
      id: index + 1,
      lat: Number(p.lat.toFixed(7)),
      lng: Number(p.lng.toFixed(7))
    }))
  };
}

function validateMission(mission, length) {
  const warnings = [];

  if (mission.controlPoints.length < 2) {
    return {
      type: 'neutral',
      text: 'Добавьте минимум две точки.'
    };
  }

  if (length > 1000) warnings.push('Маршрут длиннее 1 км.');
  if (mission.waypoints.length > 250) warnings.push('Слишком много точек для первой версии.');
  if (mission.arrivalRadiusMeters < 2) warnings.push('Радиус меньше 2 м может быть слишком строгим для GPS.');

  for (let i = 0; i < mission.controlPoints.length - 1; i++) {
    const d = distanceMeters(mission.controlPoints[i], mission.controlPoints[i + 1]);
    if (d < 2) {
      warnings.push(`Точки ${i + 1} и ${i + 2} расположены слишком близко.`);
      break;
    }
  }

  if (warnings.length) {
    return {
      type: 'warn',
      text: warnings.join(' ')
    };
  }

  return {
    type: 'good',
    text: 'Маршрут готов к экспорту. Перед запуском проверьте его на карте и используйте аварийный STOP.'
  };
}

function updateMission() {
  refreshMarkers();

  const control = getControlLatLngs();
  const spacing = Math.max(2, Number(els.spacing.value) || 8);
  const generated = generateWaypoints(control, spacing);
  const length = totalDistance(control);

  routeLine.setLatLngs(control);
  generatedLine.setLatLngs(generated);

  els.controlCount.textContent = control.length;
  els.generatedCount.textContent = generated.length;
  els.routeLength.textContent = formatDistance(length);

  const speedPercent = Math.max(5, Math.min(100, Number(els.speed.value) || 35));
  const assumedSpeed = 0.4 + (speedPercent / 100) * 1.6;
  els.estimatedTime.textContent = formatTime(length / assumedSpeed);

  els.statusText.textContent = control.length < 2
    ? 'Маршрут не создан'
    : `${control.length} контрольных точек · ${formatDistance(length)}`;

  const validation = validateMission(buildMission(), length);
  els.validation.className = `validation ${validation.type}`;
  els.validation.textContent = validation.text;
}

function clearMission() {
  controlPoints.forEach(item => map.removeLayer(item.marker));
  controlPoints.splice(0, controlPoints.length);
  updateMission();
}

function loadMission(mission) {
  if (!mission || !Array.isArray(mission.controlPoints)) {
    throw new Error('В файле нет controlPoints.');
  }

  clearMission();

  els.missionName.value = mission.missionName || 'Imported Mission';
  els.speed.value = mission.speedPercent ?? 35;
  els.arrivalRadius.value = mission.arrivalRadiusMeters ?? 3;
  els.spacing.value = mission.waypointSpacingMeters ?? 8;

  mission.controlPoints.forEach(p => {
    if (!Number.isFinite(Number(p.lat)) || !Number.isFinite(Number(p.lng))) return;
    addControlPoint(L.latLng(Number(p.lat), Number(p.lng)));
  });

  if (controlPoints.length >= 2) {
    map.fitBounds(routeLine.getBounds(), { padding: [40, 40] });
  }

  showToast('Маршрут загружен');
}

function downloadMission() {
  const mission = buildMission();

  if (mission.controlPoints.length < 2) {
    showToast('Сначала добавьте минимум две точки');
    return;
  }

  const blob = new Blob([JSON.stringify(mission, null, 2)], {
    type: 'application/json'
  });

  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  const safeName = mission.missionName.replace(/[^a-zA-Z0-9а-яА-Я_-]+/g, '_');
  a.href = url;
  a.download = `${safeName || 'mission'}.json`;
  a.click();
  URL.revokeObjectURL(url);

  showToast('mission.json скачан');
}

document.getElementById('undoBtn').addEventListener('click', () => {
  const item = controlPoints.pop();
  if (item) {
    map.removeLayer(item.marker);
    updateMission();
  }
});

document.getElementById('reverseBtn').addEventListener('click', () => {
  controlPoints.reverse();
  updateMission();
  showToast('Маршрут развернут');
});

document.getElementById('clearBtn').addEventListener('click', () => {
  if (controlPoints.length === 0 || confirm('Очистить весь маршрут?')) {
    clearMission();
  }
});

document.getElementById('centerBtn').addEventListener('click', () => {
  if (controlPoints.length >= 2) {
    map.fitBounds(routeLine.getBounds(), { padding: [40, 40] });
  } else if (controlPoints.length === 1) {
    map.setView(controlPoints[0].marker.getLatLng(), 18);
  } else {
    map.setView(DEFAULT_CENTER, 16);
  }
});

document.getElementById('exportBtn').addEventListener('click', downloadMission);

document.getElementById('copyJsonBtn').addEventListener('click', async () => {
  try {
    await navigator.clipboard.writeText(JSON.stringify(buildMission(), null, 2));
    showToast('JSON скопирован');
  } catch {
    showToast('Браузер запретил копирование');
  }
});

document.getElementById('setHomeBtn').addEventListener('click', () => {
  if (!controlPoints.length) {
    showToast('Сначала добавьте стартовую точку');
    return;
  }
  showToast('Первая точка уже используется как HOME');
});

els.importInput.addEventListener('change', async event => {
  const file = event.target.files?.[0];
  if (!file) return;

  try {
    const text = await file.text();
    loadMission(JSON.parse(text));
  } catch (error) {
    alert(`Не удалось открыть маршрут: ${error.message}`);
  } finally {
    event.target.value = '';
  }
});

[els.missionName, els.speed, els.arrivalRadius, els.spacing]
  .forEach(el => el.addEventListener('input', updateMission));

updateMission();
