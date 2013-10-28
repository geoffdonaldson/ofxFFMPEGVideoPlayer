//
//  ofxFFMPEGVideoPlayer.h
//  
//
//  Created by Geoff Donaldson on 10/23/13.
//
//

#ifndef _ofxFFMPEGVideoPlayer_h
#define _ofxFFMPEGVideoPlayer_h

#include "ofMain.h"
#include "ofPBO.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>

extern "C"{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libswscale/swscale.h>
#include <libswresample/swresample.h>
}

//// ffmpeg includes
//extern "C" {
//#define __STDC_CONSTANT_MACROS
//#include <libavcodec/avcodec.h>
//#include <libavformat/avformat.h>
//#include <libavformat/avio.h>
//#include <libswscale/swscale.h>
//    //#include <libavutil/avutil.h>
//    //#include <libavutil/rational.h>
//    //#include <libavutil/opt.h>
//}
//

//// forward declarations
//struct AVFormatContext;
//struct AVCodecContext;
//struct SwsContext;
//struct AVFrame;
//struct AVPacket;
//struct AVRational;

enum {eNoLoop, eLoop, eLoopBidi};
enum {eOpened, ePlaying, ePaused, eStopped, eEof, eError};
enum {eForward=1, eBackward=-1};
enum {eMajorVersion=0, eMinorVersion=1, ePatchVersion=0};

struct Wav
{
    short channels;
    int sampleRate;
    int sampleSize; // in bytes
    std::vector<unsigned char> data; // packed PCM sample data
};

typedef struct AudioData
{
    int						m_iSampleRate;
    int						m_iChannels;
    int						m_iSamplesCount;
    long					m_lSizeInBytes;
    long					m_lPts;
    long					m_lDts;
    unsigned char*			m_pData;
} AudioData;

typedef struct VideoData
{
    int						m_iWidth;
    int						m_iHeight;
    int						m_iChannels;
    long					m_lPts;
    long					m_lDts;
    unsigned char*			m_pData;
} VideoData;

typedef struct AVData
{
    VideoData				m_VideoData;
    AudioData				m_AudioData;
} AVData;


class ofxFFMPEGVideoPlayer /*: ofBaseVideoPlayer*/ {
    
public:
    ofxFFMPEGVideoPlayer();
    ~ofxFFMPEGVideoPlayer();
    
	bool				loadMovie(string name);
	void				close();
	void				update();
	
	void				play();
    void                pause();
	void				stop();
	
	bool 				isFrameNew();
	unsigned char * 	getPixels();
	ofTexture *			getTexture(); // if your videoplayer needs to implement seperate texture and pixel returns for performance, implement this function to return a texture instead of a pixel array. see iPhoneVideoGrabber for reference
	
	float 				getWidth();
	float 				getHeight();
	
	bool				isPaused();
	bool				isLoaded();
	bool				isPlaying();
	
	bool				setPixelFormat(ofPixelFormat pixelFormat);
	ofPixelFormat 		getPixelFormat();
    
//	//should implement!
//	virtual float 				getPosition();
//	virtual float 				getSpeed();
//	virtual float 				getDuration();
    float 				getDurationSec();

//	virtual bool				getIsMovieDone();
//	
//	virtual void 				setPaused(bool bPause);
    void 				setPosition(float pct);
//	virtual void 				setVolume(float volume); // 0..1
//	virtual void 				setLoopState(ofLoopType state);
//	virtual void   				setSpeed(float speed);
    void				setFrame(int frame);  // frame 0 = first frame...
//	
//	virtual int					getCurrentFrame();
	int					getTotalNumFrames();
    double              getFrameRate();
//	virtual ofLoopType			getLoopState();
//	
//	virtual void				firstFrame();
//	virtual void				nextFrame();
//	virtual void				previousFrame();
  
    AVData&			getAVData();
    VideoData&		getVideoData();
    AudioData&		getAudioData();
    
    std::string		getVideoCodecName();
    std::string		getAudioCodecName();
    int				getAudioChannels();
    int				getAudioSampleRate();
    float			getFps();
    float			getSpeed();
    int				getBitrate();

private:
    
    ofTexture       m_VideoTex;
    ofPBO           m_VideoPbo;
    
    bool			hasVideo();
    bool			hasAudio();
    bool			isImage();
    bool			isNewFrame();
    void			dumpFFmpegInfo();
    
    bool			openVideoStream();
    bool			openAudioStream();
    bool			seekFrame(long lFrameNumber);
    bool			seekTime(double dTimeInMs);
    bool			decodeFrame();
    bool			decodeVideoFrame(AVPacket* pAVPacket);
    bool			decodeAudioFrame(AVPacket* pAVPacket);
    bool			decodeImage();
    AVPacket*		fetchAVPacket();
    void			retrieveFileInfo();
    void			retrieveVideoInfo();
    void			retrieveAudioInfo();
    void			updateTimer();
    double			getDeltaTime();
    long			calculateFrameNumberFromTime(long lTime);
    
    double          dts_to_sec(int64_t dts);
    int64_t         dts_to_frame_number(int64_t dts);

    double			mod(double a, double b);    
    double			r2d(AVRational r);

    
    AVFormatContext*		m_pFormatContext;
    AVCodecContext*			m_pVideoCodecContext;
    AVCodecContext*			m_pAudioCodecContext;
    SwsContext*				m_pSwScalingContext;
    AVFrame*				m_pVideoFrame;
    AVFrame*				m_pVideoFrameRGB;
    AVFrame*				m_pAudioFrame;
    AVData					m_AVData;
    
    std::string				m_strFileName;
    std::string				m_strVideoCodecName;
    std::string				m_strAudioCodecName;
    unsigned char*			m_pVideoBuffer;
    unsigned char*			m_pVideoBufferYUV;
    double					m_dCurrentTimeInMs;
    double					m_dTargetTimeInMs;
    double					m_dDurationInMs;
    double					m_dFps;
    float					m_fSpeedMultiplier;			// default 1.0, no negative values
    unsigned long			m_lDurationInFrames;			// length in frames of file, or if cueIn and out are set frames between this range
    long					m_lCurrentFrameNumber;		// current framePosition ( if cue positions are set e.g. startCueFrame = 10, currentframe at absolute pos 10 is set to 0 (range between 10 and 500 --> current frame 0 .. 490)
    unsigned long			m_lFramePosInPreLoadedFile;
    long                    m_lCueInFrameNumber;   // default = 0
    long                    m_lCueOutFrameNumber;  // default = maxNrOfFrames (end of file)
    int						m_iVideoStream;
    int						m_iAudioStream;
    int						m_iContentType;				// 0 .. video with audio, 1 .. just video, 2 .. just audio, 3 .. image	// 2RealEnumeration
    int						m_iBitrate;
    int						m_iDirection;
    int						m_iLoopMode;					// 0 .. once, 1 .. loop normal, 2 .. loop bidirectional, default is loop
    int						m_iState;
    bool					m_bIsInitialized;
    bool					m_bIsFileOpen;
    bool					m_bIsThreadRunning;
    bool					isFrameDecoded;

    ofMutex                 m_Mutex;
    Poco::Timestamp         m_OldTime;
    bool                    newFrame;
    bool                    m_bHavePixelsChanged;
    
    int64_t                 m_AudioOutChannelLayout = AV_CH_LAYOUT_STEREO;
    AVSampleFormat          m_AudioOutSampleFormat = AV_SAMPLE_FMT_S16; // Packed audio, non-planar (this is the most common format, and probably what you want; also, WAV needs it)
    int                     m_AudioOutSampleRate = 44100;
    SwrContext*             m_pSwResampleContext;
    
    Wav wav;
    void saveWav(const std::string& filename, const Wav& wav);
    
};



#endif
