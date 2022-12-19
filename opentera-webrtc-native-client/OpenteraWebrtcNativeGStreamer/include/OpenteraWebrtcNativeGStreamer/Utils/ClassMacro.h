/*
 *  Copyright 2022 IntRoLab
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#ifndef OPENTERA_WEBRTC_NATIVE_GSTREAMER_UTILS_CLASS_MACRO_H
#define OPENTERA_WEBRTC_NATIVE_GSTREAMER_UTILS_CLASS_MACRO_H

#define DECLARE_NOT_COPYABLE(className)                                                                                \
    className(const className&) = delete;                                                                              \
    className& operator=(const className&) = delete

#define DECLARE_NOT_MOVABLE(className)                                                                                 \
    className(className&&) = delete;                                                                                   \
    className& operator=(className&&) = delete

#endif
