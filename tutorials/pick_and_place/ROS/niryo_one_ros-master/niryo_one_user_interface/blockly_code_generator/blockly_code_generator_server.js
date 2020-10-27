#!/usr/bin/env node

/*
    blockly_code_generator_server.js
    Copyright (C) 2017 Niryo
    All rights reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

var fs = require('fs');
var net = require('net'); 

var Blockly = require('./niryo_one_python_generators').Blockly;

Blockly.Python.STATEMENT_PREFIX = 'n.highlight_block(%1)\n';
Blockly.Python.addReservedWords('highlightBlock');

const generateCode = (dirPath) => {
    var filenameRead = dirPath + '/blockly_xml';
    var filenameWrite = dirPath + '/blockly_python';

    try {
        var xmlCode = fs.readFileSync(filenameRead, 'utf8');
    }
    catch (e) {
        return { status: 400, message: 'Could not read file : ' + filenameRead };
    }

    try {
        var xml = Blockly.Xml.textToDom(xmlCode);
    }
    catch (e) {
        return { status: 400, message: 'Could not parse XML from file : ' + filenameRead };
    }

    var workspace = new Blockly.Workspace();
    
    try {
        Blockly.Xml.domToWorkspace(xml, workspace);
    }
    catch (e) {
        console.log(e);
        return { status: 400, message: 'Failed to parse given Xml' };
    }
    
    var code = '#!/usr/bin/env python\n\nfrom niryo_one_python_api.niryo_one_api import *\n'
        + 'import rospy\nrospy.init_node(\'niryo_one_generated_code_execution\')\nn = NiryoOne()\n\n';
    
    try {
        code += Blockly.Python.workspaceToCode(workspace);
    }
    catch (e) {
        return { status: 400, message: 'Could not generate code from given Xml' };
    }

    try {
        fs.writeFileSync(filenameWrite, code, 'utf-8');
    }
    catch (e) {
        return { status: 400, message: 'Could not write generated code on file : ' + filenameWrite };
    }
   
    return  { status: 200, message: 'Successfully generated code' };
}

var HOST = '127.0.0.1';
var PORT = '1337';

var server = net.createServer(function (socket) {
    console.log('CONNECTED: ' + socket.remoteAddress + ':' + socket.remotePort);

    socket.on('data', function (dirPath) {
        var response = generateCode(dirPath.toString('utf8'));
        socket.write(JSON.stringify(response));
    });

    socket.on('close', function (data) {
        console.log('Socket connection closed... ');
    });

    socket.on('error', function (error) {
        console.log(error);
    });
}).listen(PORT, HOST);

