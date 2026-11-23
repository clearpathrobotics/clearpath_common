# Copyright 2026 Clearpath Robotics, Inc.
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

from collections import deque

import pytest

from velocity_checks import VelocityProbe


@pytest.mark.parametrize('rate', [20, 50])
def test_settled_uses_sustained_duration(rate):
    probe = object.__new__(VelocityProbe)
    target = (0.5, 0.0, 0.0)
    probe.samples = deque((index / rate, target) for index in range(rate + 1))

    assert probe.settled(target, 0.0)
    assert not probe.settled(target, 0.8)

    probe.samples[-2] = (probe.samples[-2][0], (1.0, 0.0, 0.0))
    assert not probe.settled(target, 0.0)

    probe.samples.extend((1 + index / rate, target) for index in range(1, rate + 1))
    assert probe.settled(target, 0.0)


def test_settled_requires_five_samples():
    probe = object.__new__(VelocityProbe)
    target = (0.0, 0.0, 0.0)
    probe.samples = deque((index * 0.2, target) for index in range(4))

    assert not probe.settled(target, 0.0)
