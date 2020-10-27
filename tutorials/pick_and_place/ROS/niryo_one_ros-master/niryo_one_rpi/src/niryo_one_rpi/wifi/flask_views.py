#!/usr/bin/env python

# flask_views.py
# Copyright (C) 2017 Niryo
# All rights reserved.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from flask import flash, request, jsonify, Response

from niryo_one_rpi.wifi import flask_app as app
from niryo_one_rpi.wifi.robot_name_handler import *
import network_manager as niryo_one_wifi
import json

# This id will be useful to recognize robot type (Niryo One),
ID_NIRYO_ONE = 1

# default values, will be replaced with values in setup launch file
HOTSPOT_SSID = 'Niryo_One'
HOTSPOT_PASSWORD = 'password'


def set_hotspot_ssid(ssid):
    global HOTSPOT_SSID
    HOTSPOT_SSID = ssid


def set_hotspot_password(password):
    global HOTSPOT_PASSWORD
    HOTSPOT_PASSWORD = password


def standard_response(status, message):
    response = jsonify({
        'detail': message
    })
    response.status_code = status
    return response


@app.route('/')
def index():
    message = "Instructions : \n"
    message += "/addWifi : connect to a new Wi-Fi (params : [ ssid, password ])\n"
    message += "/isItTheRealNiryoOne : todo \n"
    response = jsonify({
        'detail': message
    })
    return response


@app.route('/availableConnections', methods=['GET'])
def get_available_connections():
    connection_list = niryo_one_wifi.get_all_available_wifi()
    connection_list = filter(lambda c: c != HOTSPOT_SSID, connection_list)
    # print connection_list
    response = jsonify({
        'connections': connection_list
    })
    response.status_code = 200
    return response


@app.route('/registeredConnections', methods=['GET'])
def get_registered_connections():
    connection_list = niryo_one_wifi.get_all_registered_wifi()
    connection_list = filter(lambda c: c != HOTSPOT_SSID, connection_list)
    response = jsonify({
        'connections': connection_list
    })
    response.status_code = 200
    return response


@app.route('/restartWifi', methods=['POST'])
def restart_wifi():
    niryo_one_wifi.deactivate_current_wlan0()
    niryo_one_wifi.activate_current_wlan0()
    return standard_response(200, "Wifi has been restarted")


@app.route('/removeConnection', methods=['POST'])
def delete_connection():
    params = request.get_json()
    # print params
    if not params:
        return standard_response(400, "No ssid given")
    ssid = params.get('ssid', None)
    if ssid is None:
        return standard_response(400, "No ssid given")

    # Check if ssid = current ssid
    current_ssid = niryo_one_wifi.get_current_ssid()
    # print current_ssid
    if current_ssid == ssid:
        return standard_response(400,
                                 "Niryo One is currently connected to this ssid. Please connect the robot to another ssid, or switch to 'hotspot mode', and retry")

    if niryo_one_wifi.delete_connection_with_ssid(ssid):
        return standard_response(200, "Connection has been removed")
    else:
        return standard_response(400, "Unable to remove this connection")


@app.route('/switchToHotspot', methods=['POST'])
def switch_to_hotspot_mode():
    if niryo_one_wifi.get_current_ssid() == HOTSPOT_SSID:
        return standard_response(200, "Hotspot mode already activated")
    success = niryo_one_wifi.hard_enable_hotspot_with_ssid(HOTSPOT_SSID, HOTSPOT_PASSWORD)
    if success:
        return standard_response(200, "Hotspot mode activated")
    return standard_response(400, "Failed to activate hotspot mode")


@app.route('/addWifi', methods=['POST'])
def add_Wifi():
    params = request.get_json()
    # print params
    if not params:
        response = "Ssid or password empty"
        resp = jsonify({
            'detail': response
        })
        resp.status_code = 400
        return resp
    ssid = params.get('ssid', None)
    password = params.get('password', None)
    name = params.get('name', '')

    # print ssid, password, name
    if ssid is None or password is None:
        response = "Ssid or password empty"
        resp = jsonify({
            'detail': response
        })
        resp.status_code = 400
        return resp

    if niryo_one_wifi.connect_to_wifi(ssid, password):
        write_robot_name(str(name))
    else:
        niryo_one_wifi.hard_enable_hotspot_with_ssid(HOTSPOT_SSID, HOTSPOT_PASSWORD)
    response = "Successfully Changed Wi-Fi"
    resp = jsonify({
        'detail': response})
    resp.status_code = 200
    return resp


@app.route('/isItTheRealNiryoOne', methods=['GET'])
def isTheRealNiryoOne():
    response = jsonify({'name': str(read_robot_name())})
    resp = jsonify({
        'type': ID_NIRYO_ONE,
        'name': read_robot_name()})
    resp.status_code = 200
    return resp
