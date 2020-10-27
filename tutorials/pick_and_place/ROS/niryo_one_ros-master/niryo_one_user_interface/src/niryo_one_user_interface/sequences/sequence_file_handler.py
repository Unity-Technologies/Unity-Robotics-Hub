#!/usr/bin/env python

# sequence_file_handler.py
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

import os
import re
from threading import Lock

from niryo_one_user_interface.sequences.niryo_one_file_exception import NiryoOneFileException
from niryo_one_user_interface.sequences.sequence import Sequence


class SequenceFileHandler:

    def __init__(self, sequences_dir):
        self.base_dir = sequences_dir
        if self.base_dir.startswith('~'):
            self.base_dir = os.path.expanduser(self.base_dir)
        if not self.base_dir.endswith('/'):
            self.base_dir += '/'
        if not os.path.exists(self.base_dir):
            rospy.logwarn("Create sequences dir " + str(self.base_dir))
            os.makedirs(self.base_dir)
        self.lock = Lock()

    def read_sequence(self, identif, read_info_only=False):
        filename = self.filename_from_sequence_id(identif)
        # Check if exists
        if not self.does_file_exist(filename):
            raise NiryoOneFileException('Sequence for id ' + str(identif) + ' does not exist')
        with self.lock:
            try:
                with open(self.base_dir + filename, 'r') as f:
                    seq = Sequence()
                    for line in f:
                        if line.startswith('---ID---'):
                            seq.id = int(next(f).rstrip())
                        if line.startswith('---NAME---'):
                            seq.name = next(f).rstrip()
                        if line.startswith('---CREATED---'):
                            seq.created = int(next(f).rstrip())
                        if line.startswith('---UPDATED---'):
                            seq.updated = int(next(f).rstrip())
                        if not read_info_only and line.startswith('---DESCRIPTION---'):
                            read_next_line = True
                            while read_next_line:
                                try:
                                    new_line = next(f)
                                    if new_line.startswith('---END_OF_DESCRIPTION---'):
                                        seq.description = seq.description.rstrip("\n")
                                        read_next_line = False
                                    else:
                                        seq.description += new_line
                                except StopIteration:
                                    raise NiryoOneFileException("Malformed file - Description")
                        if not read_info_only and line.startswith('---BLOCKLY_XML---'):
                            read_next_line = True
                            while read_next_line:
                                try:
                                    new_line = next(f)
                                    if new_line.startswith('---END_OF_BLOCKLY_XML---'):
                                        seq.blockly_xml = seq.blockly_xml.rstrip("\n")
                                        read_next_line = False
                                    else:
                                        seq.blockly_xml += new_line
                                except StopIteration:
                                    raise NiryoOneFileException("Malformed file - Blockly XML")
                        if not read_info_only and line.startswith('---PYTHON_CODE---'):
                            read_next_line = True
                            while read_next_line:
                                try:
                                    new_line = next(f)
                                    if new_line.startswith('---END_OF_PYTHON_CODE---'):
                                        seq.python_code = seq.python_code.rstrip("\n")
                                        read_next_line = False
                                    else:
                                        seq.python_code += new_line
                                except StopIteration:
                                    raise NiryoOneFileException("Malformed file - Python code")
                    return seq
            except Exception as e:
                raise NiryoOneFileException("Failed to open or read from file for sequence id : " + str(identif) + str(e))

    def write_sequence(self, seq):
        filename = self.filename_from_sequence_id(seq.id)
        with self.lock:
            try:
                with open(self.base_dir + filename, 'w') as f:
                    f.write("---ID---\n")
                    f.write(str(seq.id) + "\n")
                    f.write("---NAME---\n")
                    f.write(str(seq.name) + "\n")
                    f.write("---CREATED---\n")
                    f.write(str(seq.created) + "\n")
                    f.write("---UPDATED---\n")
                    f.write(str(seq.updated) + "\n")
                    f.write("---DESCRIPTION---\n")
                    f.write(str(seq.description) + "\n")
                    f.write("---END_OF_DESCRIPTION---\n")
                    f.write("---BLOCKLY_XML---\n")
                    f.write(str(seq.blockly_xml) + "\n")
                    f.write("---END_OF_BLOCKLY_XML---\n")
                    f.write("---PYTHON_CODE---\n")
                    f.write(str(seq.python_code) + "\n")
                    f.write("---END_OF_PYTHON_CODE---\n")
            except Exception:
                raise NiryoOneFileException("Failed to write on file for sequence id : " + str(seq.id))

    def get_all_filenames(self):
        filenames = []
        try:
            filenames = os.listdir(self.base_dir)
        except OSError:
            pass
        r = re.compile("^sequence_\d+$")
        # Keep only correct filenames
        return filter(r.match, filenames)

    def does_file_exist(self, filename):
        filenames = self.get_all_filenames()
        return filename in filenames

    def remove_sequence(self, seq_id):
        filename = self.filename_from_sequence_id(seq_id)
        with self.lock:
            try:
                os.remove(self.base_dir + filename)
            except OSError as e:
                raise NiryoOneFileException("Could not remove sequence with id "
                                            + str(seq_id) + " : " + str(e))

    @staticmethod
    def sequence_id_from_filename(filename):
        return int(filename.replace('sequence_', ''))

    @staticmethod
    def filename_from_sequence_id(seq_id):
        return 'sequence_' + str(seq_id)
