
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

let devices = {
  enumerate,
  getStream
};

export default devices;
