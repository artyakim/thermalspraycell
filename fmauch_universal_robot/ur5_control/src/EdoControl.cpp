//
// Created by profanter on 8/13/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#include <EdoControl.h>

#include "EdoControl.h"

EdoControl::EdoControl(bool _isSimulation, const char* nodeHandleNamespace): isSimulation(_isSimulation), nh(nodeHandleNamespace), transformListener() {
}

EdoControl::~EdoControl() = default;