#!/usr/bin/env python

# network_manager.py
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

import os
import subprocess
import time
import re
from uuid import getnode as get_mac

HOTSPOT_MODE = False


def is_hotspot_activated():  # wifi manager (ros side) can quickly query hotspot mode
    return HOTSPOT_MODE


def get_mac_address():
    return ':'.join(("%012X" % get_mac())[i:i + 2] for i in range(0, 12, 2))


def deactivate_current_wlan0():
    """Deactivate the current wlan0 connection, if it exists"""
    print "Deactivate current wlan0..."
    count = 0
    while count < 4:
        try:
            output = subprocess.check_output([
                'sudo', 'nmcli', 'device', 'disconnect', 'wlan0'])
            print "Successfully deactivated"
            time.sleep(5)  # needed delay
            return True
        except subprocess.CalledProcessError:
            print "Fail to deactivate"
            pass
        count += 1
    return False


def activate_current_wlan0():
    """Activate the wlan0 interface"""
    print "Activate current wlan0"
    output = subprocess.check_output([
        'sudo', 'nmcli', 'device', 'status'])
    print output
    count = 0
    while count < 4:
        try:
            output = subprocess.check_output([
                'sudo', 'nmcli', 'device', 'connect', 'wlan0'])
            print output
            return True
        except subprocess.CalledProcessError:
            print "Failed"
            pass
        count += 1
    return False


def delete_connection_with_ssid(ssid):
    """Delete registered connections with corresponding ssid"""
    print "Delete connection with SSID:" + ssid
    ssid_list = get_all_registered_wifi()
    if ssid in ssid_list:
        try:
            subprocess.check_output([
                'sudo', 'nmcli', 'connection', 'delete', ssid])
            print "Connection successfully deleted"
            return True
        except subprocess.CalledProcessError:
            print "Failed to delete ssid"
            return False
    print "Can't delete an unregistered ssid"
    return False


def hard_enable_hotspot_with_ssid(ssid, passwd):
    """Create and start Hotspot with ssid and password"""
    delete_connection_with_ssid(ssid)  # will avoid duplicates
    deactivate_current_wlan0()
    activate_current_wlan0()
    retries = 4
    while retries > 0:
        output1 = subprocess.check_output([
            'sudo', 'nmcli', 'connection', 'add',
            'type', 'wifi', 'ifname', 'wlan0', 'con-name', ssid,
            'autoconnect', 'no', 'ssid', ssid, 'ip4', '10.10.10.10/24'
        ])
        # all user devices will use 10.10.10.10 when in hotspot mode
        print "Hotspot created with ip: 10.10.10.10"

        output2 = subprocess.check_output([
            'sudo', 'nmcli', 'connection', 'modify', ssid,
            '802-11-wireless.mode', 'ap', '802-11-wireless.band', 'bg',
            'ipv4.method', 'shared'
        ])

        output3 = subprocess.check_output([
            'sudo', 'nmcli', 'connection', 'modify',
            ssid, 'wifi-sec.key-mgmt', 'wpa-psk',
        ])
        print "Security set to wpa password"

        output4 = subprocess.check_output([
            'sudo', 'nmcli', 'connection', 'modify', ssid,
            'wifi-sec.psk', passwd
        ])
        print "Password set"

        count = 0
        print "Trying to start the hotspot"
        while count < 4:
            print "Attempt" + str(count)
            try:
                output5 = subprocess.check_output([
                    'sudo', 'nmcli', 'connection', 'up', 'id', ssid
                ])
                print "Hotspot successfully activated"
                global HOTSPOT_MODE
                HOTSPOT_MODE = True
                return True
            except subprocess.CalledProcessError:
                print "Error, retrying..."
                count += 1
                pass
        retries -= 1
    return False


def delete_all_duplicates_connections(ssid):
    registered_connections = get_all_registered_wifi()
    for i, c in enumerate(registered_connections):
        if c == ssid or re.match(str(ssid) + ' ' + '\d+', str(c)):
            delete_connection_with_ssid(c)


def connect_to_wifi(ssid, passwd):
    """Connect to the wifi"""
    delete_all_duplicates_connections(
        ssid)  # for each connection, network manager will create a new registered ssid with incremental number
    deactivate_current_wlan0()
    count = 0
    print "Trying to connect to wifi" + ssid
    while count < 4:
        print "Attempt number " + str(count + 1)
        try:
            output = subprocess.check_output([
                'sudo', 'nmcli', 'device', 'wifi', 'connect', ssid,
                'password', passwd])
            print "Connection successfully started"
            if is_connected_to_wifi():
                global HOTSPOT_MODE
                HOTSPOT_MODE = False
                return True
            return False
        except subprocess.CalledProcessError:
            print "Error, retrying..."
            count += 1
            pass
    return False


def is_connected_to_wifi():
    """Check if the robot is already connected to a Wifi"""
    list_enabled_connection = subprocess.Popen([
        'sudo', 'nmcli', 'connection', 'show', '--active'],
        stdout=subprocess.PIPE)
    output, error = list_enabled_connection.communicate()
    # print output
    for line in output.split(os.linesep):
        if 'wlan0' in line:
            return True
    return False


def get_current_ssid():
    """Get the ssid of the current connection ('' if no connection)"""
    list_enabled_connection = subprocess.Popen([
        'sudo', 'nmcli', 'connection', 'show', '--active'],
        stdout=subprocess.PIPE)
    output, error = list_enabled_connection.communicate()
    # print output
    for line in output.split(os.linesep):
        if 'wlan0' in line:
            return line.split('  ')[0]
    return ''


def get_all_registered_wifi():
    """Get a list of all registered SSID"""
    list_ssid = []
    list_registered_connections = subprocess.Popen([
        'sudo', 'nmcli', 'connection', 'show'],
        stdout=subprocess.PIPE)
    output, error = list_registered_connections.communicate()
    for line in output.split(os.linesep):
        if 'wireless' in line:
            list_ssid.append(line.split('  ')[0])
    return list_ssid


def is_ssid_registered(ssid):
    """Test if a SSID is registered"""
    list_ssid = get_all_registered_wifi()
    return ssid in list_ssid


def get_all_available_wifi():
    """Get all currently available Wifi"""
    list_available_wifi = []
    subprocess.check_output([
        'sudo', 'nmcli', 'device', 'wifi', 'rescan'])
    output, error = subprocess.Popen([
        'sudo', 'nmcli', 'device', 'wifi', 'list'],
        stdout=subprocess.PIPE).communicate()
    for line in output.split(os.linesep):
        line = line[3:]
        if line != '':
            if (line.split('  ', 2)[0] != "SSID" and
                    line.split('  ', 2)[0] != '--'):
                # print line
                list_available_wifi.append(line.split('  ', 2)[0])
    # print list_available_wifi
    list_available_wifi = list(set(list_available_wifi))  # remove duplicates
    return list_available_wifi


def is_ssid_available(ssid):
    """Check if the ssid is """
    list_available_wifi = get_all_available_wifi()
    return ssid in list_available_wifi


def get_registered_and_available_connection():
    """Get a list of all registered and available Wifi"""
    available_wifi_list = get_all_available_wifi()
    registered_wifi_list = get_all_registered_wifi()
    return [item for item in registered_wifi_list
            if item in available_wifi_list]
