
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

async function getStream(constraints) {
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    throw new Error('getUserMedia() is not supported.');
  }

  return await navigator.mediaDevices.getUserMedia(constraints);
}

async function getDefaultStream() {
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    throw new Error('getUserMedia() is not supported.');
  }

  return await navigator.mediaDevices.getUserMedia({ audio: true, video: true });
}

async function getDefaultAudioStream() {
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    throw new Error('getUserMedia() is not supported.');
  }

  return await navigator.mediaDevices.getUserMedia({ audio: true, video: false });
}

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
