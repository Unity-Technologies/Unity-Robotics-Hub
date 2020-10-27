#!/usr/bin/env python

import os
import json

from niryo_one_commander.niryo_one_file_exception import NiryoOneFileException


class FileManager(object):
    """
    Manages the creation, storage and loading of objects that implement the functions
    from_dict and to_dict.

    :raises NiryoOneFileException:
    """

    def __init__(self, base_dir, prefix, protected_names=None):
        self.__base_dir = os.path.expanduser(base_dir)
        self.__protected_names = protected_names if protected_names is not None else []
        if not self.__base_dir.endswith('/'):
            self.__base_dir += '/'
        if not os.path.exists(self.__base_dir):
            os.makedirs(self.__base_dir)

        self.__prefix = prefix

    def __name_from_filename(self, filename):
        if filename.startswith(self.__prefix + '_'):
            return filename[len(self.__prefix + '_'):]
        else:
            return filename

    def __filename_from_name(self, name):
        return self.__prefix + '_' + name

    def __path_from_name(self, name):
        return self.__base_dir + self.__filename_from_name(name)

    def _write(self, name, object_):
        if name in self.__protected_names:
            raise NiryoOneFileException("Object {} is protected and cannot be written".format(name))
        with open(self.__path_from_name(name), 'w') as f:
            try:
                f.write(json.dumps(object_.to_dict(), f, indent=2))
            except Exception as e:
                raise NiryoOneFileException("Could not write object. " + str(e))

    def read(self, name):
        if not self.exists(name):
            raise NiryoOneFileException("Object " + str(name) + ' does not exist')

        with open(self.__path_from_name(name), 'r') as f:
            try:
                return self.object_type.from_dict(json.loads(f.read()))
            except Exception as e:
                raise NiryoOneFileException("Could not read object " + name + " : " + str(e))

    def remove(self, name):
        if name in self.__protected_names:
            raise NiryoOneFileException("Object {} is protected and cannot be removed".format(name))
        try:
            os.remove(self.__path_from_name(name))
        except OSError as e:
            raise NiryoOneFileException("Could not remove object " + name + " : " + str(e))

    def get_all_names(self):
        try:
            filenames = os.listdir(self.__base_dir)
        except OSError as e:
            raise NiryoOneFileException("Could not retrieve files. " + str(e))
        return [self.__name_from_filename(f) for f in filenames if f.startswith(self.__prefix)]

    def exists(self, name):
        path = self.__path_from_name(name)
        return os.path.isfile(path)


if __name__ == '__main__':
    pass
