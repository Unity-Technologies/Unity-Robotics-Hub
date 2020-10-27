#!/usr/bin/env python

# sequence_manager.py
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

import rospy

from niryo_one_msgs.msg import Sequence as SequenceMessage
from niryo_one_msgs.srv import GetSequenceList
from niryo_one_msgs.srv import ManageSequence

from niryo_one_user_interface.sequences.sequence import Sequence
from niryo_one_user_interface.sequences.niryo_one_file_exception import NiryoOneFileException
from niryo_one_user_interface.sequences.sequence_file_handler import SequenceFileHandler
from niryo_one_user_interface.sequences.sequence_command_type import SequenceCommandType
from niryo_one_user_interface.sequences.blockly_code_generator import BlocklyCodeGenerator


class SequenceManager:

    def __init__(self, sequences_dir):
        self.fh = SequenceFileHandler(sequences_dir)
        self.blockly_generator = BlocklyCodeGenerator()

        self.get_sequence_list_server = rospy.Service(
            '/niryo_one/sequences/get_sequence_list', GetSequenceList, self.callback_get_sequence_list)

        self.manage_command_server = rospy.Service(
            '/niryo_one/sequences/manage_sequence', ManageSequence, self.callback_manage_sequence)

    def get_python_code_from_xml(self, xml):
        return self.blockly_generator.get_generated_python_code(xml)

    # !! Need to call this with rospy.on_shutdown !!
    def shutdown(self):
        self.blockly_generator.shutdown()

    def callback_get_sequence_list(self, req):
        read_info_only = req.info_header_only
        sequence_list = self.get_all_sequences(read_info_only=read_info_only)
        msg_list = []
        for seq in sequence_list:
            msg = SequenceMessage()
            msg.id = seq.id
            msg.name = seq.name
            msg.description = seq.description
            msg.created = seq.created
            msg.updated = seq.updated
            msg.blockly_xml = seq.blockly_xml
            msg.python_code = seq.python_code
            msg_list.append(msg)
        return {'sequences': msg_list}

    @staticmethod
    def create_sequence_response(status, message, sequence=None):
        seq_msg = SequenceMessage()
        if sequence is not None:
            seq_msg.id = sequence.id
            seq_msg.name = sequence.name
            seq_msg.description = sequence.description
            seq_msg.created = sequence.created
            seq_msg.updated = sequence.updated
            seq_msg.blockly_xml = sequence.blockly_xml
            seq_msg.python_code = sequence.python_code
        return {'status': status, 'message': message, 'sequence': seq_msg}

    def callback_manage_sequence(self, req):
        cmd_type = req.cmd_type
        seq_id = req.sequence_id
        seq_msg = req.sequence
        sequence_data = SequenceMessage(id=0, name=seq_msg.name, description=seq_msg.description,
                                 blockly_xml=seq_msg.blockly_xml, python_code=seq_msg.python_code)

        # GET sequence from id
        if cmd_type == SequenceCommandType.GET:
            seq = self.get_sequence_from_id(seq_id)
            if seq is None:
                return self.create_sequence_response(400, "No sequence found with id : " + str(seq_id))
            return self.create_sequence_response(200, "Sequence has been found", seq)

        # CREATE new sequence
        elif cmd_type == SequenceCommandType.CREATE:
            new_seq_id = self.save_new_sequence(sequence_data)
            new_seq = self.get_sequence_from_id(new_seq_id)
            if new_seq is None:
                return self.create_sequence_response(400, "Failed to create sequence")
            return self.create_sequence_response(200, "Sequence has been created", new_seq)

        # UPDATE existing sequence
        elif cmd_type == SequenceCommandType.UPDATE:
            seq = self.get_sequence_from_id(seq_id)
            if seq is None:
                return self.create_sequence_response(400, "No sequence found with id : " + str(seq_id))
            success = self.update_sequence(seq, sequence_data)
            if not success:
                return self.create_sequence_response(400, "Could not update sequence with id : " + str(seq_id))
            return self.create_sequence_response(200, "Sequence has been updated", seq)

        # DELETE sequence
        elif cmd_type == SequenceCommandType.DELETE:
            success = self.delete_sequence(seq_id)
            if not success:
                return self.create_sequence_response(400, "Could not delete sequence with id : " + str(seq_id))
            return self.create_sequence_response(200, "Sequence has been deleted")

        # GET last executed
        elif cmd_type == SequenceCommandType.GET_LAST_EXECUTED:
            seq = self.get_last_executed_sequence()
            if seq is None:
                return self.create_sequence_response(400, "No last executed sequence has been found")
            return self.create_sequence_response(200, "Sequence has been found", seq)

        # Wrong cmd_type 
        else:
            return self.create_sequence_response(400, "Wrong command type")

    def get_sequence_from_id(self, seq_id, read_info_only=False):
        try:
            return self.fh.read_sequence(seq_id, read_info_only=read_info_only)
        except NiryoOneFileException as e:
            return None

    def get_all_sequences(self, read_info_only=False):
        filenames = self.fh.get_all_filenames()
        sequence_list = []
        for f in filenames:
            try:
                sequence_id = self.fh.sequence_id_from_filename(f)
                sequence = self.get_sequence_from_id(sequence_id, read_info_only=read_info_only)
                if sequence is not None:
                    sequence_list.append(sequence)
            except NiryoOneFileException as e:
                pass
        return sequence_list

    def save_new_sequence(self, sequence):
        identif = self.pick_new_id()
        sequence.id = identif
        if sequence.name == "":
            sequence.name = self.fh.filename_from_sequence_id(identif)
        sequence.created = rospy.Time.now().secs
        sequence.updated = rospy.Time.now().secs
        try:
            self.fh.write_sequence(sequence)
        except NiryoOneFileException as e:
            return -1
        return identif

    # choose a non used, incremental id
    def pick_new_id(self):
        filenames = self.fh.get_all_filenames()
        max_id = 0
        for filename in filenames:
            current_id = self.fh.sequence_id_from_filename(filename)
            if current_id > max_id:
                max_id = current_id
        return max_id + 1

    def update_sequence(self, sequence, sequence_data):
        sequence.name = sequence_data.name
        sequence.description = sequence_data.description
        sequence.blockly_xml = sequence_data.blockly_xml
        sequence.python_code = sequence_data.python_code
        sequence.updated = rospy.Time.now().secs
        try:
            self.fh.write_sequence(sequence)
        except NiryoOneFileException as e:
            return False
        return True

    def delete_sequence(self, identif):
        try:
            self.fh.remove_sequence(identif)
        except NiryoOneFileException as e:
            return False
        return True

    def get_last_executed_sequence(self, read_info_only=False):
        return self.get_sequence_from_id(0, read_info_only=read_info_only)

    def save_last_executed_sequence(self, sequence):
        sequence.id = 0
        sequence.name = "Last executed command"
        sequence.created = rospy.Time.now().secs
        sequence.updated = rospy.Time.now().secs
        try:
            self.fh.write_sequence(sequence)
        except NiryoOneFileException as e:
            return -1
        return 0


if __name__ == '__main__':
    # rospy.init_node('niryo_one_sequence_manager')
    # s = SequenceManager('/home/edouard/sequences_niryo')
    # rospy.on_shutdown(s.on_shutdown);
    # rospy.spin()
    pass
