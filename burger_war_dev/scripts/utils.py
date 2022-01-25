#!/usr/bin/env python
# -*- coding: utf-8 -*-

import csv

def readCsv(path):
    with open(path, "r") as f:
        csvreader = csv.reader(f)
        ret = []
        for row in csvreader:
            ret.append([float(val) for val in row])

    return ret
