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
    </div>
  </div>

    <VideoView
      ref="webcam"
      selectFirstDevice="true"
      :name="name"
      :deviceId="deviceId"
      @cameras="camera_list"
      @camera-change="onCameraChange"
    />
  </div>
</template>

<script>
// @ is an alias to /src
import VideoView from '@/components/VideoView.vue'

export default {
  name: 'Home',
  components: {
    VideoView
  },
  data () {
    return {
      img: null,
      camera: null,
      deviceId: null,
      name: null,
      devices: []
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
    onCameraChange (deviceId) {
      this.deviceId = deviceId
      this.camera = deviceId
      console.log('On Camera Change Event', deviceId)
    }
  }
}
</script>
