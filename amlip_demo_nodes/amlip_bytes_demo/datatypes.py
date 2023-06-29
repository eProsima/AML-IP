# Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from amlip_py.types.JobSolutionDataType import JobSolutionDataType
from amlip_py.types.JobDataType import JobDataType
import pickle


class Book():

    def __init__(
            self,
            filename):
        self.filename = filename

    def tobytes(self) -> bytearray:
        with open(self.filename, 'rb') as file:
            return bytearray(file.read())


def get_keys_with_max_values(dictionary, n):
    sorted_items = sorted(dictionary.items(), key=lambda x: x[1], reverse=True)
    max_keys = [item for item in sorted_items[:n]]
    return max_keys


class FileStatistics():

    def __init__(
            self,
            txt: str):

        # Count number of lines
        self.num_lines = len(txt.split('\n'))

        # Count number of chars
        self.size_bytes = len(txt)

        # Count appearance of each work
        self.word_counts = {}
        for word in txt.lower().split():
            self.word_counts[word] = self.word_counts.get(word, 0) + 1

    def tobytes(self) -> bytes:
        pickle.dumps(self)

    def __str__(self) -> str:
        st = 'FileStatistics{'
        st += f'lines:{self.num_lines}'
        st += f' ; bytes:{self.size_bytes} ; '
        st += str(get_keys_with_max_values(self.word_counts, 3))
        return st
