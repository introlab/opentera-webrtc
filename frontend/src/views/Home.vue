<template>
  <div class="home">

  <div class="row">
    <div class="col-md-12">
    <select v-model="deviceId">
      <option>-- Select Device --</option>
      <option v-for="device in devices" :key="device.deviceId" :value="device.deviceId">
        {{ device.label }}
      </option>
    </select>
    </div>

    <div class="col-md-12">
      <button type="button" class="btn btn-danger" @click="onStop">Stop Camera</button>
      <button type="button" class="btn btn-success" @click="onStart">Start Camera</button>
      <button type="button" class="btn btn-success" @click="onJoin">Join</button>
    </div>
    <div>
      <vue-webrtc ref="webrtc" width="100%" roomId="roomId" socketURL="https://localhost:9001/" :enableLogs="logEnabled" @>
      </vue-webrtc>
    </div>
  </div>
<!--
    <VideoView
      ref="webcam"
      :name="name"
      :deviceId="deviceId"
      @cameras="camera_list"
      @camera-change="onCameraChange"
      @started="onCameraStarted"
    />
-->
  </div>
</template>

<script>
// @ is an alias to /src
// import VideoView from '@/components/VideoView.vue'
import WebRTC from 'vue-webrtc'
import Vue from 'vue'
// ISSUE 5: https://github.com/westonsoftware/vue-webrtc/issues/5
import * as io from 'socket.io-client'
window.io = io

Vue.use(WebRTC)
// Vue.component(WebRTC.name, WebRTC)

export default {
  name: 'Home',
  components: {
    // VideoView
  },
  data () {
    return {
      img: null,
      camera: null,
      deviceId: null,
      name: null,
      devices: [],
      logEnabled: true
    }
  },
  methods: {
    camera_list (cams) {
      // Store devices list
      this.devices = cams
      console.log('camera_list', cams)
    },
    onCapture () {
      this.img = this.$refs.webcam.capture()
    },
    onStop () {
      this.$refs.webcam.stop()
    },
    onStart () {
      this.$refs.webcam.start()
    },
    onCameraStarted () {
      console.log('Camera Started')
    },
    onCameraChange (deviceId) {
      this.deviceId = deviceId
      this.camera = deviceId
      console.log('On Camera Change Event', deviceId)
    },
    onJoin () {
      console.log('On join')
      this.$refs.webrtc.join()
    }
  }
}
</script>
