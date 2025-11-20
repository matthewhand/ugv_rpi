export const socketAudio = io('http://' + location.host + '/audio');
export const socketJson = io('http://' + location.host + '/json');
export const socketCtrl = io('http://' + location.host + '/ctrl');

export const fetchConfig ='/config';

export const fetchUploadAudio ='/upload_audio';
export const fetchGetAudioFiles ='/get_audio_files';
export const fetchPlayAudio ='/play_audio';
export const fetchStopAudio ='/stop_audio';
export const fetchDeleteAudio ='/delete_audio';

export const fetchGetPhotoNames ='/get_photo_names';
export const fetchDeletePhoto ='/delete_photo';

export const fetchGetVideoNames ='/get_video_names';
export const fetchDeleteVideo ='/delete_video';

export const fetchSendCommand ='/send_command';




