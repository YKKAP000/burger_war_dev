#!/usr/bin/env python
# -*- coding: utf-8 -*-

from utils import readCsv

goals = readCsv("./input/strategy.csv")
for goal in goals:
    print(goal)
