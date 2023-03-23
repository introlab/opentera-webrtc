/**
 * @brief Enumerates all media devices.
 * @returns {Promise<MediaDeviceInfo[]>}
 */
async function enumerate() {
  if (!navigator.mediaDevices || !navigator.mediaDevices.enumerateDevices) {
    throw new Error('enumerateDevices() is not supported.');
  }
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    throw new Error('getUserMedia() is not supported.');
  }

  await navigator.mediaDevices.getUserMedia({ audio: true, video: true });
  return await navigator.mediaDevices.enumerateDevices();
}

/**
 * @brief Gets a stream that satisfies the constraints.
 * @param {Object} constraints See https://developer.mozilla.org/en-US/docs/Web/API/MediaDevices/getUserMedia
 * @returns {Promise<MediaStream>}
 */
async function getStream(constraints) {
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    throw new Error('getUserMedia() is not supported.');
  }

  return await navigator.mediaDevices.getUserMedia(constraints);
}

/**
 * @brief Gets the default audio and video stream.
 * @returns {Promise<MediaStream>}
 */
async function getDefaultStream() {
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    throw new Error('getUserMedia() is not supported.');
  }

  return await navigator.mediaDevices.getUserMedia({ audio: true, video: true });
}

/**
 * @brief Gets the default audio stream.
 * @returns {Promise<MediaStream>}
 */
async function getDefaultAudioStream() {
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    throw new Error('getUserMedia() is not supported.');
  }

  return await navigator.mediaDevices.getUserMedia({ audio: true, video: false });
}

/**
 * @brief Gets the default video stream.
 * @returns {Promise<MediaStream>}
 */
async function getDefaultVideoStream() {
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    throw new Error('getUserMedia() is not supported.');
  }

  return await navigator.mediaDevices.getUserMedia({ audio: false, video: true });
}

let devices = {
  enumerate,
  getStream,
  getDefaultStream,
  getDefaultAudioStream,
  getDefaultVideoStream
};

export default devices;
