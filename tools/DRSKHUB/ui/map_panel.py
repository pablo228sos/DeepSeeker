from __future__ import annotations
import json
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt6.QtCore import pyqtSignal, QUrl, Qt
try:
    from PyQt6.QtWebEngineWidgets import QWebEngineView
    from PyQt6.QtWebEngineCore import QWebEngineSettings
    HAS_WEBENGINE = True
except ImportError:
    HAS_WEBENGINE = False

MAP_HTML = """
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8"/>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
html,body,#map{margin:0;padding:0;width:100%;height:100%;background:#0d111a;}
.leaflet-tile-pane{filter:brightness(0.85) saturate(0.9);}
.drone-label{background:rgba(10,11,13,0.9);color:#e2e8f0;border:1px solid #21262f;border-radius:4px;padding:2px 6px;font-size:11px;font-family:Inter,sans-serif;white-space:nowrap;}
.wp-label{background:#3b82f6;color:#fff;border-radius:50%;width:20px;height:20px;display:flex;align-items:center;justify-content:center;font-size:11px;font-weight:700;}
</style>
</head>
<body>
<div id="map"></div>
<script>
var map=L.map("map",{center:[41.0082,28.9784],zoom:14,zoomControl:false,attributionControl:false});
var dark=L.tileLayer("https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png",{maxZoom:19,subdomains:"abcd"});
var osm=L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",{maxZoom:19});
var sat=L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",{maxZoom:19});
dark.addTo(map);var curLayer=dark;
function setLayer(n){map.removeLayer(curLayer);curLayer=n==="osm"?osm:n==="sat"?sat:dark;curLayer.addTo(map);}
var droneM={},droneT={},dronePL={};
function updateDrone(id,lat,lon,hdg,color,name,bat,alt,sel){
  var sz=sel?38:30;
  var icon=L.divIcon({className:"",html:"<div style=\"transform:rotate("+hdg+"deg);font-size:"+sz+"px;color:"+color+";filter:drop-shadow(0 0 6px "+color+"99);line-height:1;\">\u2708</div>",iconSize:[sz,sz],iconAnchor:[sz/2,sz/2]});
  if(!droneM[id]){droneM[id]=L.marker([lat,lon],{icon:icon}).addTo(map);dronePL[id]=L.polyline([],{color:color,weight:1.5,opacity:0.45}).addTo(map);droneT[id]=[];droneM[id].on("click",function(){if(window.droneClick)window.droneClick(id);});}
  else{droneM[id].setLatLng([lat,lon]);droneM[id].setIcon(icon);}
  droneM[id].bindTooltip("<div class=\"drone-label\"><b>"+name+"</b><br/>ALT "+alt.toFixed(0)+"m  BAT "+bat+"%</div>",{permanent:false,direction:"right",offset:[14,0]});
  droneT[id].push([lat,lon]);if(droneT[id].length>400)droneT[id].shift();dronePL[id].setLatLngs(droneT[id]);
}
function removeDrone(id){if(droneM[id]){map.removeLayer(droneM[id]);delete droneM[id];}if(dronePL[id]){map.removeLayer(dronePL[id]);delete dronePL[id];}}
function clearTracks(){Object.keys(dronePL).forEach(function(id){dronePL[id].setLatLngs([]);droneT[id]=[];});}
var wpM=[],wpLine=L.polyline([],{color:"#3b82f6",weight:1.5,dashArray:"8 4",opacity:0.7}).addTo(map);
function setWaypoints(wps){wpM.forEach(function(m){map.removeLayer(m);});wpM=[];var ll=[];wps.forEach(function(wp,i){var icon=L.divIcon({className:"",html:"<div class=\"wp-label\">"+(i+1)+"</div>",iconSize:[20,20],iconAnchor:[10,10]});var m=L.marker([wp.lat,wp.lon],{icon:icon}).addTo(map);m.bindTooltip(wp.type+" ALT:"+wp.alt+"m",{direction:"top"});wpM.push(m);ll.push([wp.lat,wp.lon]);});wpLine.setLatLngs(ll);}
var fenceL=null;
function setFence(lat,lon,r,en){if(fenceL){map.removeLayer(fenceL);fenceL=null;}if(en)fenceL=L.circle([lat,lon],{radius:r,color:"#ef4444",weight:2,dashArray:"8 4",fillColor:"#ef4444",fillOpacity:0.04}).addTo(map);}
map.on("click",function(e){if(window.mapClick)window.mapClick(e.latlng.lat,e.latlng.lng);});
function centerAll(lat,lon,z){map.setView([lat,lon],z||14);}
function zoomIn(){map.zoomIn();}function zoomOut(){map.zoomOut();}
</script>
</body>
</html>
"""

class MapPanel(QWidget):
    map_clicked   = pyqtSignal(float, float)
    drone_clicked = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._fence_center  = (41.0082, 28.9784)
        self._fence_radius  = 500
        self._fence_enabled = False
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        tb_widget = QWidget()
        tb_widget.setStyleSheet("background:#111318;border-bottom:1px solid #21262f;")
        tb_widget.setFixedHeight(36)
        tb = QHBoxLayout(tb_widget)
        tb.setContentsMargins(8, 4, 8, 4)
        tb.setSpacing(6)
        for label, tip, cb in [
            ("+", "Zoom in", self.zoom_in),
            ("-", "Zoom out", self.zoom_out),
            ("Center", "Center fleet", self.center_all),
            ("Dark", "Dark map", lambda: self.set_layer("dark")),
            ("Street", "Street map", lambda: self.set_layer("osm")),
            ("Satellite", "Satellite", lambda: self.set_layer("sat")),
            ("Fence", "Toggle fence", self._toggle_fence),
            ("Clear Tracks", "Clear tracks", self.clear_tracks),
        ]:
            btn = QPushButton(label)
            btn.setToolTip(tip)
            btn.setFixedHeight(26)
            btn.setStyleSheet("font-size:11px;padding:2px 10px;")
            btn.clicked.connect(cb)
            tb.addWidget(btn)
        tb.addStretch()
        self._coord_lbl = QLabel("Click map to place waypoint")
        self._coord_lbl.setStyleSheet("font-size:11px;color:#475569;font-family:Consolas,monospace;")
        tb.addWidget(self._coord_lbl)
        layout.addWidget(tb_widget)

        if HAS_WEBENGINE:
            self._view = QWebEngineView()
            s = self._view.settings()
            s.setAttribute(QWebEngineSettings.WebAttribute.JavascriptEnabled, True)
            s.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True)
            self._view.setHtml(MAP_HTML, QUrl("about:blank"))
            layout.addWidget(self._view, 1)
        else:
            self._view = None
            ph = QLabel("Install PyQt6-WebEngine for live map:\npip install PyQt6-WebEngine")
            ph.setAlignment(Qt.AlignmentFlag.AlignCenter)
            ph.setStyleSheet("color:#475569;font-size:14px;")
            layout.addWidget(ph, 1)

    def _js(self, code):
        if self._view:
            self._view.page().runJavaScript(code)

    def update_drone(self, drone_id, lat, lon, heading, color, name, bat, alt, selected=False):
        if lat == 0.0 and lon == 0.0:
            return
        sel = "true" if selected else "false"
        c = json.dumps(color); n = json.dumps(name); did = json.dumps(drone_id)
        self._js(f"updateDrone({did},{lat},{lon},{heading},{c},{n},{bat},{alt},{sel});")
        if selected:
            self._coord_lbl.setText(f"{name}  {lat:.5f}N  {lon:.5f}E")

    def remove_drone(self, drone_id):
        self._js(f"removeDrone({json.dumps(drone_id)});")

    def set_waypoints(self, wps):
        self._js(f"setWaypoints({json.dumps(wps)});")

    def center_on(self, lat, lon, zoom=14):
        self._js(f"centerAll({lat},{lon},{zoom});")

    def center_all(self):
        self.center_on(*self._fence_center)

    def set_layer(self, name):
        self._js(f"setLayer({json.dumps(name)});")

    def set_fence(self, lat, lon, radius, enabled):
        self._fence_center = (lat, lon)
        self._fence_radius = radius
        self._fence_enabled = enabled
        en = "true" if enabled else "false"
        self._js(f"setFence({lat},{lon},{radius},{en});")

    def clear_tracks(self):
        self._js("clearTracks();")

    def zoom_in(self):  self._js("zoomIn();")
    def zoom_out(self): self._js("zoomOut();")

    def _toggle_fence(self):
        self._fence_enabled = not self._fence_enabled
        lat, lon = self._fence_center
        self.set_fence(lat, lon, self._fence_radius, self._fence_enabled)