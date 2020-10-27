/*
    niryo_one_python_generators.js
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

// adds Custom Niryo One blocks + Python generators

var Blockly = require('blockly');
var niryo_one_color = "#3D4D9A";

// Interface color
var logic_color = "#00876d";
var loop_color = "#49a563";
var math_color = "#5769a1";
var list_color = "#765da1";
var variable_color = "#ad5a7e";
var function_color = "#9f5ca1";
var movement_color = "#4f87c0";
var io_color = "#c05150";
var tool_color = "#bf964b";
var utility_color = "#bead76";
var vision_color = "#546e7a";
var conveyor_color = "#00838f";

// Color object for vision
//TODO Should be in a class
const g_color_values = {
    'COLOR_RED': "RED",
    'COLOR_GREEN': "GREEN",
    'COLOR_BLUE': "BLUE",
    'COLOR_ANY': "ANY"
}

// Shape object for vision
//TODO Should be in a class
const g_shape_values = {
    'SHAPE_SQUARE': "SQUARE",
    'SHAPE_CIRCLE': "CIRCLE",
    'SHAPE_ANY': "ANY"
}

/*
 *  Blocks definition
 */

// Movement

Blockly.Blocks['niryo_one_move_joints'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("Move Joints");
        this.appendDummyInput()
            .appendField("j1")
            .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.001), "JOINTS_1")
            .appendField("j2")
            .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.001), "JOINTS_2")
            .appendField("j3")
            .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.001), "JOINTS_3")
            .appendField("j4")
            .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.001), "JOINTS_4")
            .appendField("j5")
            .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.001), "JOINTS_5")
            .appendField("j6")
            .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.001), "JOINTS_6");
        this.setInputsInline(true);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(movement_color);
        this.setTooltip("Give all 6 joints to move the robot");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_move_pose'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("Move Pose");
        this.appendDummyInput()
            .appendField("x")
            .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.001), "POSE_X")
            .appendField("y")
            .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.001), "POSE_Y")
            .appendField("z")
            .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.001), "POSE_Z")
            .appendField("roll")
            .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.001), "POSE_ROLL")
            .appendField("pitch")
            .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.001), "POSE_PITCH")
            .appendField("yaw")
            .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.001), "POSE_YAW");
        this.setInputsInline(true);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(movement_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_shift_pose'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("Shift");
        this.appendDummyInput()
            .appendField(new Blockly.FieldDropdown([["pos. x", "0"], ["pos. y", "1"], ["pos. z", "2"], ["rot. x", "3"], ["rot. y", "4"], ["rot. z", "5"]]), "SHIFT_POSE_AXIS")
            .appendField("by")
            .appendField(new Blockly.FieldNumber(0, -Infinity, Infinity, 0.001), "SHIFT_POSE_VALUE");
        this.setInputsInline(true);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(movement_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_set_arm_max_speed'] = {
    init: function () {
        this.appendValueInput("SET_ARM_MAX_SPEED")
            .setCheck("Number")
            .appendField("Set Arm max. speed to");
        this.appendDummyInput()
            .appendField("%");
        this.setInputsInline(true);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(movement_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_calibrate_auto'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("Calibrate motors (auto)");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(movement_color);
        this.setTooltip("Will auto calibrate motors. If already calibrated, will do nothing.");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_calibrate_manual'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("Calibrate motors (manual)");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(movement_color);
        this.setTooltip("Will manually calibrate motors (robot needs to be in home position). If already calibrated, will do nothing.");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_activate_learning_mode'] = {
    init: function () {
        this.appendDummyInput()
            .appendField(new Blockly.FieldDropdown([["Activate", "1"], ["Deactivate", "0"]]), "LEARNING_MODE_VALUE")
            .appendField("learning mode");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(movement_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_joint'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("Joints");
        this.appendValueInput("j1")
            .setCheck("Number")
            .appendField("j1");
        this.appendValueInput("j2")
            .setCheck("Number")
            .appendField("j2");
        this.appendValueInput("j3")
            .setCheck("Number")
            .appendField("j3");
        this.appendValueInput("j4")
            .setCheck("Number")
            .appendField("j4");
        this.appendValueInput("j5")
            .setCheck("Number")
            .appendField("j5");
        this.appendValueInput("j6")
            .setCheck("Number")
            .appendField("j6");
        this.setInputsInline(true);
        this.setOutput(true, null);
        this.setColour(movement_color);
        this.setTooltip("Represents an object pose");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_move_joint_from_joint'] = {
    init: function () {
        this.appendValueInput("JOINT")
            .setCheck("niryo_one_joint")
            .appendField("Move joint");
            this.setTooltip("Move joint with an object pose given");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(movement_color);
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_move_pose_from_pose'] = {
    init: function () {
        this.appendValueInput("POSE")
            .setCheck("niryo_one_pose")
            .appendField("Move pose");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(movement_color);
        this.setTooltip("Move pose with an object pose given");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_pose'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("Pose");
        this.appendValueInput("x")
            .setCheck("Number")
            .appendField("x");
        this.appendValueInput("y")
            .setCheck("Number")
            .appendField("y");
        this.appendValueInput("z")
            .setCheck("Number")
            .appendField("z");
        this.appendValueInput("roll")
            .setCheck("Number")
            .appendField("roll");
        this.appendValueInput("pitch")
            .setCheck("Number")
            .appendField("pitch");
        this.appendValueInput("yaw")
            .setCheck("Number")
            .appendField("yaw");
        this.setInputsInline(true);
        this.setOutput(true, null);
        this.setColour(movement_color);
        this.setTooltip("Represents an object pose");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_move_pose_from_pose'] = {
    init: function () {
        this.appendValueInput("POSE")
            .setCheck("niryo_one_pose")
            .appendField("Move pose");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(movement_color);
        this.setTooltip("Move pose with an object pose given");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_pick_from_pose'] = {
    init: function () {
        this.appendValueInput("POSE")
            .setCheck("niryo_one_pose")
            .appendField("Pick from pose");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(movement_color);
        this.setTooltip("Pick an object at a pose given");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_place_from_pose'] = {
    init: function () {
        this.appendValueInput("POSE")
            .setCheck("niryo_one_pose")
            .appendField("Place from pose");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(movement_color);
        this.setTooltip("Place an object at a pose given");
        this.setHelpUrl("");
    }
};

// I/O

Blockly.Blocks['niryo_one_gpio_select'] = {
    init: function () {
        this.appendDummyInput()
            .appendField(new Blockly.FieldDropdown([["1A", "GPIO_1A"], ["1B", "GPIO_1B"], ["1C", "GPIO_1C"], ["2A", "GPIO_2A"], ["2B", "GPIO_2B"], ["2C", "GPIO_2C"]]), "GPIO_SELECT");
        this.setOutput(true, "niryo_one_gpio_select");
        this.setColour(io_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_set_pin_mode'] = {
    init: function () {
        this.appendValueInput("SET_PIN_MODE_PIN")
            .setCheck("niryo_one_gpio_select")
            .setAlign(Blockly.ALIGN_RIGHT)
            .appendField("Set Pin");
        this.appendDummyInput()
            .appendField("to mode")
            .appendField(new Blockly.FieldDropdown([["INPUT", "PIN_MODE_INPUT"], ["OUTPUT", "PIN_MODE_OUTPUT"]]), "PIN_MODE_SELECT");
        this.setInputsInline(true);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(io_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_digital_write'] = {
    init: function () {
        this.appendValueInput("DIGITAL_WRITE_PIN")
            .setCheck("niryo_one_gpio_select")
            .appendField("Set Pin");
        this.appendDummyInput()
            .appendField("to state")
            .appendField(new Blockly.FieldDropdown([["HIGH", "PIN_HIGH"], ["LOW", "PIN_LOW"]]), "PIN_WRITE_SELECT");
        this.setInputsInline(true);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(io_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_digital_read'] = {
    init: function () {
        this.appendValueInput("DIGITAL_READ_PIN")
            .setCheck("niryo_one_gpio_select")
            .appendField("Get Pin");
        this.appendDummyInput()
            .appendField("state");
        this.setInputsInline(true);
        this.setOutput(true, "niryo_one_gpio_state");
        this.setColour(io_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_gpio_state'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("state")
            .appendField(new Blockly.FieldDropdown([["HIGH", "PIN_HIGH"], ["LOW", "PIN_LOW"]]), "GPIO_STATE_SELECT");
        this.setOutput(true, "niryo_one_gpio_state");
        this.setColour(io_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_sw_select'] = {
    init: function () {
        this.appendDummyInput()
            .appendField(new Blockly.FieldDropdown([["SW1", "SW_1"], ["SW2", "SW_2"]]), "SW_SELECT");
        this.setOutput(true, "niryo_one_sw_select");
        this.setColour(io_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_set_12v_switch'] = {
    init: function () {
        this.appendValueInput("SET_12V_SWITCH")
            .setCheck("niryo_one_sw_select")
            .appendField("Set 12V Switch");
        this.appendDummyInput()
            .appendField("to state")
            .appendField(new Blockly.FieldDropdown([["HIGH", "PIN_HIGH"], ["LOW", "PIN_LOW"]]), "SET_12V_SWITCH_SELECT");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(io_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

// Tool

Blockly.Blocks['niryo_one_tool_select'] = {
    init: function () {
        this.appendDummyInput()
            .appendField(new Blockly.FieldDropdown([["Standard gripper", "TOOL_GRIPPER_1_ID"], ["Large gripper", "TOOL_GRIPPER_2_ID"], ["Adaptive gripper ", "TOOL_GRIPPER_3_ID"], ["electromagnet 1", "TOOL_ELECTROMAGNET_1_ID"], ["vacuum pump 1", "TOOL_VACUUM_PUMP_1_ID"]]), "TOOL_SELECT");
        this.setOutput(true, "niryo_one_tool_select");
        this.setColour(tool_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_change_tool'] = {
    init: function () {
        this.appendValueInput("NEW_TOOL_ID")
            .setCheck("niryo_one_tool_select")
            .appendField("Change tool to");
        this.setInputsInline(true);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(tool_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_detach_tool'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("Detach current tool");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(tool_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_open_gripper'] = {
    init: function () {
        this.appendValueInput("OPEN_GRIPPER_ID")
            .setCheck("niryo_one_tool_select")
            .appendField("Open Gripper");
        this.appendDummyInput()
            .appendField("at speed")
            .appendField(new Blockly.FieldDropdown([["1/5", "100"], ["2/5", "250"], ["3/5", "500"], ["4/5", "750"], ["5/5", "1000"]]), "OPEN_SPEED");
        this.setInputsInline(true);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(tool_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_close_gripper'] = {
    init: function () {
        this.appendValueInput("CLOSE_GRIPPER_ID")
            .setCheck("niryo_one_tool_select")
            .appendField("Close Gripper");
        this.appendDummyInput()
            .appendField("at speed")
            .appendField(new Blockly.FieldDropdown([["1/5", "100"], ["2/5", "250"], ["3/5", "500"], ["4/5", "750"], ["5/5", "1000"]]), "CLOSE_SPEED");
        this.setInputsInline(true);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(tool_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_pull_air_vacuum_pump'] = {
    init: function () {
        this.appendValueInput("PULL_AIR_VACUUM_PUMP_ID")
            .setCheck("niryo_one_tool_select")
            .appendField("Pull air with Vacuum Pump");
        this.setInputsInline(true);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(tool_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_push_air_vacuum_pump'] = {
    init: function () {
        this.appendValueInput("PUSH_AIR_VACUUM_PUMP_ID")
            .setCheck("niryo_one_tool_select")
            .appendField("Push air with Vacuum Pump");
        this.setInputsInline(true);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(tool_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_setup_electromagnet'] = {
    init: function () {
        this.appendValueInput("SETUP_ELECTROMAGNET_ID")
            .setCheck("niryo_one_tool_select")
            .appendField("Setup Electromagnet");
        this.appendValueInput("SETUP_ELECTROMAGNET_PIN")
            .setCheck("niryo_one_gpio_select")
            .setAlign(Blockly.ALIGN_RIGHT)
            .appendField("with pin");
        this.setInputsInline(true);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(tool_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_activate_electromagnet'] = {
    init: function () {
        this.appendValueInput("ACTIVATE_ELECTROMAGNET_ID")
            .setCheck("niryo_one_tool_select")
            .appendField("Activate Electromagnet");
        this.appendValueInput("ACTIVATE_ELECTROMAGNET_PIN")
            .setCheck("niryo_one_gpio_select")
            .setAlign(Blockly.ALIGN_RIGHT)
            .appendField("with pin");
        this.setInputsInline(true);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(tool_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_deactivate_electromagnet'] = {
    init: function () {
        this.appendValueInput("DEACTIVATE_ELECTROMAGNET_ID")
            .setCheck("niryo_one_tool_select")
            .appendField("Deactivate Electromagnet");
        this.appendValueInput("DEACTIVATE_ELECTROMAGNET_PIN")
            .setCheck("niryo_one_gpio_select")
            .setAlign(Blockly.ALIGN_RIGHT)
            .appendField("with pin");
        this.setInputsInline(true);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(tool_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

// Utility

Blockly.Blocks['niryo_one_sleep'] = {
    init: function () {
        this.appendValueInput("SLEEP_TIME")
            .setCheck("Number")
            .appendField("Wait for ");
        this.appendDummyInput()
            .appendField("seconds");
        this.setInputsInline(true);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(utility_color);
        this.setTooltip("");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_comment'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("Comment :")
            .appendField(new Blockly.FieldTextInput(""), "COMMENT_TEXT");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(utility_color);
        this.setTooltip("This block will not be executed.");
        this.setHelpUrl("");
    }
};

Blockly.Blocks['niryo_one_break_point'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("Break Point");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(utility_color);
        this.setTooltip("Stop the execution of the program. Press 'Play' to resume.");
        this.setHelpUrl("");
    }
};

// Vision

Blockly.Blocks['niryo_one_vision_color'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("Color")
            .appendField(new Blockly.FieldDropdown([["RED", "COLOR_RED"],
            ["GREEN", "COLOR_GREEN"],
            ["BLUE", "COLOR_BLUE"],
            ["ANY", "COLOR_ANY"]]), "COLOR_SELECT");
        this.setOutput(true, "niryo_one_vision_color");
        this.setColour(vision_color);
        this.setTooltip("Color object (must be used with Vision's blocks)");
    }
}

Blockly.Blocks['niryo_one_vision_shape'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("Shape")
            .appendField(new Blockly.FieldDropdown([["SQUARE", "SHAPE_SQUARE"],
            ["CIRCLE", "SHAPE_CIRCLE"],
            ["OBJECT", "SHAPE_ANY"]]), "SHAPE_SELECT");
        this.setOutput(true, "niryo_one_vision_shape");
        this.setColour(vision_color);
        this.setTooltip("Shape object (must be used with Vision's blocks)");
    }
}


Blockly.Blocks['niryo_one_vision_pick'] = {
    init: function () {
        this.appendValueInput("COLOR_SWITCH")
            .setCheck("niryo_one_vision_color")
            .appendField("Vision pick");

        this.appendValueInput("SHAPE_SWITCH")
            .setCheck("niryo_one_vision_shape");
        this.appendDummyInput().appendField("in workspace");

	this.appendValueInput("WORKSPACE_NAME")
            .setCheck("String");

        this.appendValueInput("HEIGHT_OFFSET")
            .setCheck("Number")
            .appendField("with height offset (mm)");


        this.setOutput(true, "Boolean");
        this.setColour(vision_color);
        this.setHelpUrl("");
        this.setTooltip("Pick an object of SHAPE / COLOR  given, with gripper close position at HEIGHT_OFFSET cm.");
        this.setInputsInline(false);
    }
}

Blockly.Blocks['niryo_one_vision_is_object_detected'] = {
    init: function () {
        this.appendValueInput("COLOR_SWITCH")
            .setCheck("niryo_one_vision_color")
            .appendField("Is object detected");

        this.appendValueInput("SHAPE_SWITCH")
            .setCheck("niryo_one_vision_shape");
        this.appendDummyInput().appendField("in workspace");

        this.appendValueInput("WORKSPACE_NAME")
            .setCheck("String");

        this.setOutput(true, "Boolean");
        this.setColour(vision_color);
        this.setHelpUrl("");
        this.setTooltip("Detect is there is an object of SHAPE / COLOR in the WORKSPACE given.");
        this.setInputsInline(false);
    }
}

// Conveyor

Blockly.Blocks['niryo_one_conveyor_models'] = {
    init: function () {
        this.appendDummyInput()
            .appendField(new Blockly.FieldDropdown([["Conveyor 1", "CONVEYOR_1"],
            ["Conveyor 2", "CONVEYOR_2"]]), "CONVEYOR_SELECT");
        this.setOutput(true, "niryo_one_conveyor_models");

        this.setColour(conveyor_color);
        this.setHelpUrl("");
        this.setTooltip("Conveyors available with Niryo One.");
    }
}

Blockly.Blocks['niryo_one_conveyor_use'] = {
    init: function () {
        this.appendValueInput("CONVEYOR_SWITCH")
            .setCheck("niryo_one_conveyor_models")
            .appendField("Use conveyor:");

        this.setColour(conveyor_color);
        this.setHelpUrl("");
        this.setTooltip("Allow the conveyor to be controlled via Niryo One.");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
    }
}

Blockly.Blocks['niryo_one_conveyor_control'] = {
    init: function () {
        this.appendValueInput("CONVEYOR_SWITCH")
            .setCheck("niryo_one_conveyor_models")
            .appendField("Control conveyor:");

        this.appendValueInput("SPEED_PERCENT")
            .setCheck("Number")
            .appendField("with speed (%):")

        this.appendDummyInput()
            .appendField("in direction:")
            .appendField(new Blockly.FieldDropdown([["FORWARD", "1"],
            ["BACKWARD", "-1"]]), "DIRECTION_SELECT");

        this.setColour(conveyor_color);
        this.setHelpUrl("");
        this.setTooltip("Control the conveyor given.");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setInputsInline(false);
    }
}

Blockly.Blocks['niryo_one_conveyor_stop'] = {
    init: function () {
        this.appendValueInput("CONVEYOR_SWITCH")
            .setCheck("niryo_one_conveyor_models")
            .appendField("Stop conveyor");

        this.setColour(conveyor_color);
        this.setHelpUrl("");
        this.setTooltip("Stop the conveyor given.");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
    }
}

/*
 * Generators
 */

// Movement

Blockly.Python['niryo_one_move_joints'] = function (block) {
    var number_joints_1 = block.getFieldValue('JOINTS_1');
    var number_joints_2 = block.getFieldValue('JOINTS_2');
    var number_joints_3 = block.getFieldValue('JOINTS_3');
    var number_joints_4 = block.getFieldValue('JOINTS_4');
    var number_joints_5 = block.getFieldValue('JOINTS_5');
    var number_joints_6 = block.getFieldValue('JOINTS_6');
  
    var code = 'n.move_joints([' + number_joints_1 + ', ' + number_joints_2 + ', '
      + number_joints_3 + ', ' + number_joints_4 + ', ' + number_joints_5 + ', ' + number_joints_6 + '])\n'
    return code;
  };
  
  Blockly.Python['niryo_one_move_pose'] = function (block) {
    var number_pose_x = block.getFieldValue('POSE_X');
    var number_pose_y = block.getFieldValue('POSE_Y');
    var number_pose_z = block.getFieldValue('POSE_Z');
    var number_pose_roll = block.getFieldValue('POSE_ROLL');
    var number_pose_pitch = block.getFieldValue('POSE_PITCH');
    var number_pose_yaw = block.getFieldValue('POSE_YAW');
  
    var code = 'n.move_pose(' + number_pose_x + ', ' + number_pose_y + ', ' +
      number_pose_z + ', ' + number_pose_roll + ', ' + number_pose_pitch +
      ', ' + number_pose_yaw + ")\n";
    return code;
  };
  
  Blockly.Python['niryo_one_shift_pose'] = function (block) {
    var dropdown_shift_pose_axis = block.getFieldValue('SHIFT_POSE_AXIS');
    var number_shift_pose_value = block.getFieldValue('SHIFT_POSE_VALUE');
  
    var code = 'n.shift_pose(' + dropdown_shift_pose_axis + ', ' +
      number_shift_pose_value + ')\n';
    return code;
  };
  
  Blockly.Python['niryo_one_set_arm_max_speed'] = function (block) {
    var value_set_arm_max_speed = Blockly.Python.valueToCode(block, 'SET_ARM_MAX_SPEED', Blockly.Python.ORDER_ATOMIC) || '0';
    value_set_arm_max_speed = value_set_arm_max_speed.replace('(', '').replace(')', '');
    var code = 'n.set_arm_max_velocity(' + value_set_arm_max_speed + ')\n';
    return code;
  };
  
  Blockly.Python['niryo_one_calibrate_auto'] = function (block) {
    var code = 'n.calibrate_auto()\n';
    return code;
  };
  
  Blockly.Python['niryo_one_calibrate_manual'] = function (block) {
    var code = 'n.calibrate_manual()\n';
    return code;
  };
  
  Blockly.Python['niryo_one_activate_learning_mode'] = function (block) {
    var dropdown_learning_mode_value = block.getFieldValue('LEARNING_MODE_VALUE');
    var code = 'n.activate_learning_mode(' + dropdown_learning_mode_value + ')\n';
    return code;
  };
  
  
  Blockly.Python['niryo_one_joint'] = function (block) {
    var value_j1 = Blockly.Python.valueToCode(block, 'j1', Blockly.Python.ORDER_ATOMIC).replace('(', '').replace(')', '');
    var value_j2 = Blockly.Python.valueToCode(block, 'j2', Blockly.Python.ORDER_ATOMIC).replace('(', '').replace(')', '');
    var value_j3 = Blockly.Python.valueToCode(block, 'j3', Blockly.Python.ORDER_ATOMIC).replace('(', '').replace(')', '');
    var value_j4 = Blockly.Python.valueToCode(block, 'j4', Blockly.Python.ORDER_ATOMIC).replace('(', '').replace(')', '');
    var value_j5 = Blockly.Python.valueToCode(block, 'j5', Blockly.Python.ORDER_ATOMIC).replace('(', '').replace(')', '');
    var value_j6 = Blockly.Python.valueToCode(block, 'j6', Blockly.Python.ORDER_ATOMIC).replace('(', '').replace(')', '');
  
    var code = "[" + value_j1 + ", " + value_j2 + ", " + value_j3 + ", " + value_j4 + ", " + value_j5 + ", " + value_j6 + "]";
    return [code, Blockly.Python.ORDER_NONE];
  };
  
  Blockly.Python['niryo_one_move_joint_from_joint'] = function (block) {
    // Position object
    var value_joint =  Blockly.Python.valueToCode(block, 'JOINT', Blockly.Python.ORDER_ATOMIC);
    value_joint = value_joint.replace('(', '').replace(')', '');
  
    var code = 'n.move_joints(' + value_joint + ')\n';
    return code;
  }
  
  Blockly.Python['niryo_one_pose'] = function (block) {
    var value_x = Blockly.Python.valueToCode(block, 'x', Blockly.Python.ORDER_ATOMIC).replace('(', '').replace(')', '');
    var value_y = Blockly.Python.valueToCode(block, 'y', Blockly.Python.ORDER_ATOMIC).replace('(', '').replace(')', '');
    var value_z = Blockly.Python.valueToCode(block, 'z', Blockly.Python.ORDER_ATOMIC).replace('(', '').replace(')', '');
    var value_roll = Blockly.Python.valueToCode(block, 'roll', Blockly.Python.ORDER_ATOMIC).replace('(', '').replace(')', '');
    var value_pitch = Blockly.Python.valueToCode(block, 'pitch', Blockly.Python.ORDER_ATOMIC).replace('(', '').replace(')', '');
    var value_yaw = Blockly.Python.valueToCode(block, 'yaw', Blockly.Python.ORDER_ATOMIC).replace('(', '').replace(')', '');
  
    var code = "[" + value_x + ", " + value_y + ", " + value_z + ", " + value_roll + ", " + value_pitch + ", " + value_yaw + "]";
    return [code, Blockly.Python.ORDER_NONE];
  };
  
  Blockly.Python['niryo_one_move_pose_from_pose'] = function (block) {
    // Position object
    var value_pose =  Blockly.Python.valueToCode(block, 'POSE', Blockly.Python.ORDER_ATOMIC);
    value_pose = value_pose.replace('(', '').replace(')', '');
  
    var code = 'n.move_pose(*' + value_pose + ')\n';
    return code;
  }
  
  Blockly.Python['niryo_one_pick_from_pose'] = function (block) {
    // Position object
    var value_pose = Blockly.Python.valueToCode(block, 'POSE', Blockly.Python.ORDER_ATOMIC);
    value_pose = value_pose.replace('(', '').replace(')', '');
  
    var code = 'n.pick_from_pose(*' + value_pose + ')\n';
    return code;
  };
  
  Blockly.Python['niryo_one_place_from_pose'] = function (block) {
    // Position object
    var value_pose =  Blockly.Python.valueToCode(block, 'POSE', Blockly.Python.ORDER_ATOMIC);
    value_pose = value_pose.replace('(', '').replace(')', '');
  
    var code = 'n.place_from_pose(*' + value_pose + ')\n';
    return code;
  }
  
  // I/O
  
  Blockly.Python['niryo_one_gpio_state'] = function (block) {
    var dropdown_gpio_state_select = block.getFieldValue('GPIO_STATE_SELECT');
    var code = dropdown_gpio_state_select;
    return [code, Blockly.Python.ORDER_NONE];
  };
  
  Blockly.Python['niryo_one_set_pin_mode'] = function (block) {
    var value_pin = Blockly.Python.valueToCode(block, 'SET_PIN_MODE_PIN', Blockly.Python.ORDER_ATOMIC) || '(0)';
    value_pin = value_pin.replace('(', '').replace(')', '');
    var dropdown_pin_mode_select = block.getFieldValue('PIN_MODE_SELECT');
    var code = 'n.pin_mode(' + value_pin + ', ' + dropdown_pin_mode_select + ')\n';
    return code;
  };
  
  Blockly.Python['niryo_one_digital_write'] = function (block) {
    var value_pin = Blockly.Python.valueToCode(block, 'DIGITAL_WRITE_PIN', Blockly.Python.ORDER_ATOMIC) || '(0)';
    value_pin = value_pin.replace('(', '').replace(')', '');
    var dropdown_pin_write_select = block.getFieldValue('PIN_WRITE_SELECT');
    var code = 'n.digital_write(' + value_pin + ', ' + dropdown_pin_write_select + ')\n';
    return code;
  };
  
  Blockly.Python['niryo_one_digital_read'] = function (block) {
    var value_pin = Blockly.Python.valueToCode(block, 'DIGITAL_READ_PIN', Blockly.Python.ORDER_ATOMIC) || '(0)';
    value_pin = value_pin.replace('(', '').replace(')', '');
    var code = 'n.digital_read(' + value_pin + ')';
    return [code, Blockly.Python.ORDER_NONE];
  };
  
  Blockly.Python['niryo_one_gpio_select'] = function (block) {
    var dropdown_gpio_select = block.getFieldValue('GPIO_SELECT');
    var code = dropdown_gpio_select;
    return [code, Blockly.Python.ORDER_NONE];
  };
  
  Blockly.Python['niryo_one_sw_select'] = function (block) {
    var dropdown_sw_select = block.getFieldValue('SW_SELECT');
    var code = dropdown_sw_select;
    return [code, Blockly.Python.ORDER_NONE];
  };
  
  Blockly.Python['niryo_one_set_12v_switch'] = function (block) {
    var value_pin = Blockly.Python.valueToCode(block, 'SET_12V_SWITCH', Blockly.Python.ORDER_ATOMIC) || '(0)';
    value_pin = value_pin.replace('(', '').replace(')', '');
    var dropdown_set_12v_switch_select = block.getFieldValue('SET_12V_SWITCH_SELECT');
    var code = 'n.digital_write(' + value_pin + ', ' + dropdown_set_12v_switch_select + ')\n';
    return code;
  };
  
  // Tool
  
  Blockly.Python['niryo_one_tool_select'] = function (block) {
    var dropdown_tool_select = block.getFieldValue('TOOL_SELECT');
    var code = dropdown_tool_select;
    return [code, Blockly.Python.ORDER_NONE];
  };
  
  Blockly.Python['niryo_one_change_tool'] = function (block) {
    var value_tool_name = Blockly.Python.valueToCode(block, 'NEW_TOOL_ID', Blockly.Python.ORDER_ATOMIC) || '(TOOL_NONE)';
    value_tool_name = value_tool_name.replace('(', '').replace(')', '');
    var code = 'n.change_tool(' + value_tool_name + ')\n';
    return code;
  };
  
  Blockly.Python['niryo_one_detach_tool'] = function (block) {
    var code = 'n.change_tool(0)\n';
    return code;
  };
  
  Blockly.Python['niryo_one_open_gripper'] = function (block) {
    var value_gripper_id = Blockly.Python.valueToCode(block, 'OPEN_GRIPPER_ID', Blockly.Python.ORDER_ATOMIC) || '(TOOL_NONE)';
    value_gripper_id = value_gripper_id.replace('(', '').replace(')', '');
    var number_open_speed = block.getFieldValue('OPEN_SPEED');
    var code = 'n.open_gripper(' + value_gripper_id + ', ' + number_open_speed + ')\n';
    return code;
  };
  
  Blockly.Python['niryo_one_close_gripper'] = function (block) {
    var value_gripper_id = Blockly.Python.valueToCode(block, 'CLOSE_GRIPPER_ID', Blockly.Python.ORDER_ATOMIC) || '(TOOL_NONE)';
    value_gripper_id = value_gripper_id.replace('(', '').replace(')', '');
    var number_close_speed = block.getFieldValue('CLOSE_SPEED');
    var code = 'n.close_gripper(' + value_gripper_id + ', ' + number_close_speed + ')\n';
    return code;
  };
  
  Blockly.Python['niryo_one_pull_air_vacuum_pump'] = function (block) {
    var value_vacuum_pump_id = Blockly.Python.valueToCode(block, 'PULL_AIR_VACUUM_PUMP_ID', Blockly.Python.ORDER_ATOMIC) || '(TOOL_NONE)';
    value_vacuum_pump_id = value_vacuum_pump_id.replace('(', '').replace(')', '');
    var code = 'n.pull_air_vacuum_pump(' + value_vacuum_pump_id + ')\n';
    return code;
  };
  
  Blockly.Python['niryo_one_push_air_vacuum_pump'] = function (block) {
    var value_vacuum_pump_id = Blockly.Python.valueToCode(block, 'PUSH_AIR_VACUUM_PUMP_ID', Blockly.Python.ORDER_ATOMIC) || '(TOOL_NONE)';
    value_vacuum_pump_id = value_vacuum_pump_id.replace('(', '').replace(')', '');
    var code = 'n.push_air_vacuum_pump(' + value_vacuum_pump_id + ')\n';
    return code;
  };
  
  Blockly.Python['niryo_one_setup_electromagnet'] = function (block) {
    var value_electromagnet_id = Blockly.Python.valueToCode(block, 'SETUP_ELECTROMAGNET_ID', Blockly.Python.ORDER_ATOMIC) || '(TOOL_NONE)';
    value_electromagnet_id = value_electromagnet_id.replace('(', '').replace(')', '');
    var value_electromagnet_pin = Blockly.Python.valueToCode(block, 'SETUP_ELECTROMAGNET_PIN', Blockly.Python.ORDER_ATOMIC) || '(0)';
    value_electromagnet_pin = value_electromagnet_pin.replace('(', '').replace(')', '');
    var code = 'n.setup_electromagnet(' + value_electromagnet_id + ', ' + value_electromagnet_pin + ')\n';
    return code;
  };
  
  Blockly.Python['niryo_one_activate_electromagnet'] = function (block) {
    var value_electromagnet_id = Blockly.Python.valueToCode(block, 'ACTIVATE_ELECTROMAGNET_ID', Blockly.Python.ORDER_ATOMIC) || '(TOOL_NONE)';
    value_electromagnet_id = value_electromagnet_id.replace('(', '').replace(')', '');
    var value_electromagnet_pin = Blockly.Python.valueToCode(block, 'ACTIVATE_ELECTROMAGNET_PIN', Blockly.Python.ORDER_ATOMIC) || '(0)';
    value_electromagnet_pin = value_electromagnet_pin.replace('(', '').replace(')', '');
    var code = 'n.activate_electromagnet(' + value_electromagnet_id + ', ' + value_electromagnet_pin + ')\n';
    return code;
  };
  
  Blockly.Python['niryo_one_deactivate_electromagnet'] = function (block) {
    var value_electromagnet_id = Blockly.Python.valueToCode(block, 'DEACTIVATE_ELECTROMAGNET_ID', Blockly.Python.ORDER_ATOMIC) || '(TOOL_NONE)';
    value_electromagnet_id = value_electromagnet_id.replace('(', '').replace(')', '');
    var value_electromagnet_pin = Blockly.Python.valueToCode(block, 'DEACTIVATE_ELECTROMAGNET_PIN', Blockly.Python.ORDER_ATOMIC) || '(0)';
    value_electromagnet_pin = value_electromagnet_pin.replace('(', '').replace(')', '');
    var code = 'n.deactivate_electromagnet(' + value_electromagnet_id + ', ' + value_electromagnet_pin + ')\n';
    return code;
  };
  
  // Utility
  
  Blockly.Python['niryo_one_sleep'] = function (block) {
    var value_sleep_time = Blockly.Python.valueToCode(block, 'SLEEP_TIME', Blockly.Python.ORDER_ATOMIC) || '0';
    value_sleep_time = value_sleep_time.replace('(', '').replace(')', '');
    var code = 'n.wait(' + value_sleep_time + ')\n';
    return code;
  };
  
  Blockly.Python['niryo_one_comment'] = function (block) {
    var text_comment_text = block.getFieldValue('COMMENT_TEXT');
    var code = ' #' + text_comment_text + '\n';
    return code;
  };
  
  Blockly.Python['niryo_one_break_point'] = function (block) {
    var code = 'n.break_point()\n';
    return code;
  };
  
  // Vision
  
  Blockly.Python['niryo_one_vision_color'] = function (block) {
    var dropdown_color_select = block.getFieldValue('COLOR_SELECT');
    var code = dropdown_color_select;
    code = "\"" + g_color_values[code] + "\"";
    return [code, Blockly.Python.ORDER_NONE];
  };
  
  Blockly.Python['niryo_one_vision_shape'] = function (block) {
    var dropdown_shape_select = block.getFieldValue('SHAPE_SELECT');
    var code = dropdown_shape_select;
    code = "\"" + g_shape_values[code] + "\"";
    return [code, Blockly.Python.ORDER_NONE];
  };
  
  Blockly.Python['niryo_one_vision_pick'] = function (block) {
    // Color (int) value (see g_shape_values at top of this file)
    var value_color = Blockly.Python.valueToCode(block, 'COLOR_SWITCH', Blockly.Python.ORDER_ATOMIC) || '(0)';
    value_color = value_color.replace('(', '').replace(')', '');
  
    // Shape (int) value (see g_shape_values at top of this file)
    var value_shape = Blockly.Python.valueToCode(block, 'SHAPE_SWITCH', Blockly.Python.ORDER_ATOMIC) || '(0)';
    value_shape = value_shape.replace('(', '').replace(')', '');
  
    // Name of workspace
    var workspace_name = Blockly.Python.valueToCode(block, 'WORKSPACE_NAME', Blockly.Python.ORDER_ATOMIC) || '(0)';
    workspace_name = workspace_name.replace('(', '').replace(')', '');

    // Height in centimeter        
    var height_offset = Blockly.Python.valueToCode(block, 'HEIGHT_OFFSET', Blockly.Python.ORDER_ATOMIC) || '(0)';
    height_offset = height_offset.replace('(', '').replace(')', '');                                                                                                 
  
    var code = 'n.vision_pick(' + workspace_name + ', float(' + height_offset + ')/1000, ' + value_shape + ', ' + value_color + ')[0]';
    return [code, Blockly.Python.ORDER_NONE];
  }
  
  Blockly.Python['niryo_one_vision_is_object_detected'] = function (block) {
    // Color (int) value (see g_shape_values at top of this file)
    var value_color = Blockly.Python.valueToCode(block, 'COLOR_SWITCH', Blockly.Python.ORDER_ATOMIC) || '(0)';
    value_color = value_color.replace('(', '').replace(')', '');
  
    // Shape (int) value (see g_shape_values at top of this file)
    var value_shape = Blockly.Python.valueToCode(block, 'SHAPE_SWITCH', Blockly.Python.ORDER_ATOMIC) || '(0)';
    value_shape = value_shape.replace('(', '').replace(')', '');
  
    // Name of workspace
    var workspace_name = Blockly.Python.valueToCode(block, 'WORKSPACE_NAME', Blockly.Python.ORDER_ATOMIC) || '(0)';
    workspace_name = workspace_name.replace('(', '').replace(')', '');
  
    var code = 'n.detect_object(' + workspace_name + ', ' + value_shape + ', ' + value_color + ')[0]';
    return [code, Blockly.Python.ORDER_NONE];
  }
  
  // Conveyor
  
  Blockly.Python['niryo_one_conveyor_models'] = function (block) {
    const conveyor_id_map = {
      "CONVEYOR_1": 6,
      "CONVEYOR_2": 7
    };
    var conveyor_model_id = block.getFieldValue('CONVEYOR_SELECT');
    var code = conveyor_id_map[conveyor_model_id];
    return [code, Blockly.Python.ORDER_NONE];
  }
  
  Blockly.Python['niryo_one_conveyor_use'] = function (block) {
    var conveyor_id = Blockly.Python.valueToCode(block, 'CONVEYOR_SWITCH', Blockly.Python.ORDER_ATOMIC) || '(0)';
    conveyor_id = conveyor_id.replace('(', '').replace(')', '');
    var code = 'n.set_conveyor(' + conveyor_id + ', True)\n';
    return code;
  }
  
  Blockly.Python['niryo_one_conveyor_control'] = function (block) {
    var conveyor_id = Blockly.Python.valueToCode(block, 'CONVEYOR_SWITCH', Blockly.Python.ORDER_ATOMIC) || '(0)';
    conveyor_id = conveyor_id.replace('(', '').replace(')', '');
    var speed_percent = Blockly.Python.valueToCode(block, 'SPEED_PERCENT', Blockly.Python.ORDER_ATOMIC) || '(0)';
    speed_percent = speed_percent.replace('(', '').replace(')', '');
    var direction = block.getFieldValue('DIRECTION_SELECT');
    var code = 'n.control_conveyor(' + conveyor_id + ', True, ' + speed_percent + ', ' + direction + ')\n';
    return code;
  }
  
  Blockly.Python['niryo_one_conveyor_stop'] = function (block) {
    var conveyor_id = Blockly.Python.valueToCode(block, 'CONVEYOR_SWITCH', Blockly.Python.ORDER_ATOMIC) || '(0)';
    conveyor_id = conveyor_id.replace('(', '').replace(')', '');
    var code = 'n.control_conveyor(' + conveyor_id + ', False, 0, 1)\n';
    return code;
  }

/*
 * Export module
 */

module.exports = {
    Blockly: Blockly,
};
