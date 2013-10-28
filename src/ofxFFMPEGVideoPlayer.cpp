//
//  ofxFFMPEGVideoPlayer.cpp
//  
//
//  Created by Geoff Donaldson on 10/23/13.
//
//

#include "ofxFFMPEGVideoPlayer.h"

#define EPS 0.000025	// epsilon for checking unsual results as taken from OpenCV FFmeg player

ofxFFMPEGVideoPlayer::ofxFFMPEGVideoPlayer()
{
    // init property variables
	m_bIsFileOpen = false;
	m_bIsThreadRunning = false;
	m_pFormatContext = NULL;
	m_pVideoCodecContext = NULL;
	m_pAudioCodecContext = NULL;
	m_pSwScalingContext = NULL;
	m_pVideoFrame = NULL;
	m_pVideoFrameRGB = NULL;
	m_pVideoBuffer = NULL;
	m_pAudioFrame = NULL;
	
	m_iLoopMode = eLoop;
	m_dTargetTimeInMs = 0;
	m_lCurrentFrameNumber = -1;	// set to invalid, as it is not decoded yet
	m_dCurrentTimeInMs = -1;	// set to invalid, as it is not decoded yet
	m_fSpeedMultiplier = 1.0;
	m_dFps = 0;
	m_iBitrate = 0;
	m_lDurationInFrames = 0;
	m_dDurationInMs = 0;
	m_iDirection = eForward;
	m_iState = eStopped;
	m_lFramePosInPreLoadedFile = 0;
    m_lCueInFrameNumber = -1;
    
	m_AVData.m_VideoData.m_iWidth = 0;
	m_AVData.m_VideoData.m_iHeight = 0;
	m_AVData.m_VideoData.m_pData = NULL;
	m_AVData.m_VideoData.m_lPts = 0;
	m_AVData.m_VideoData.m_lDts = 0;
	m_AVData.m_VideoData.m_iChannels = 0;
    
	m_AVData.m_AudioData.m_iChannels = 0;
	m_AVData.m_AudioData.m_iSampleRate = 0;
	m_AVData.m_AudioData.m_lSizeInBytes = 0;
	m_AVData.m_AudioData.m_lPts = 0;
	m_AVData.m_AudioData.m_lDts = 0;
	m_AVData.m_AudioData.m_pData = NULL;
    
    
}

ofxFFMPEGVideoPlayer::~ofxFFMPEGVideoPlayer()
{
//    saveWav("/Users/geoff/Desktop/out.wav", wav);
    close();
}

bool ofxFFMPEGVideoPlayer::loadMovie(string name)
{

	m_strFileName = name;
    
	if(m_bIsFileOpen)
	{
		stop();
		close();
	}

    av_register_all();
    avformat_network_init();
    av_log_set_level(AV_LOG_ERROR);
    
	// Open video file
	if(avformat_open_input(&m_pFormatContext, name.c_str(), NULL, NULL)!=0)
        return false; // couldn't open file
    
	// Retrieve stream information
	if(av_find_stream_info(m_pFormatContext)<0)
		return false; // couldn't find stream information
    
	// Find the first video stream
	m_iVideoStream = m_iAudioStream = -1;
	for(unsigned int i=0; i<m_pFormatContext->nb_streams; i++)
	{
        if((m_iVideoStream < 0) && (m_pFormatContext->streams[i]->codec->codec_type==AVMEDIA_TYPE_VIDEO))
        {
            m_iVideoStream=i;
        }
        if((m_iAudioStream < 0) && (m_pFormatContext->streams[i]->codec->codec_type==AVMEDIA_TYPE_AUDIO))
        {
            m_iAudioStream=i;
        }
	}
    
	if(!(hasVideo() || hasAudio()))
        return false; // Didn't find video or audio stream
    
	if(hasVideo())
	{
		if(!openVideoStream())
			return false;
	}
    
	if(hasAudio())
	{
		if(!openAudioStream())
			return false;
	}
    
	// general file info equal for audio and video stream
	retrieveFileInfo();
    
	m_bIsFileOpen = true;
	m_strFileName = name;
    
	// content is image, just decode once
	if(isImage())
	{
		m_dDurationInMs = 0;
		m_dFps = 0;
		m_lCurrentFrameNumber = 1;
		decodeImage();
	}
    
    m_VideoTex.allocate(getWidth(), getHeight(), GL_RGB);
    m_VideoPbo.allocate(m_VideoTex, 1);
    
	// start timer
	m_OldTime.update();
    
    newFrame = false;
    m_bHavePixelsChanged = false;
    
	return m_bIsFileOpen;
}

void ofxFFMPEGVideoPlayer::close()
{
    stop();
    
	// Free the RGB image
	if(m_pVideoBuffer!=NULL)
	{
		delete m_pVideoBuffer;
		m_pVideoBuffer = NULL;
	}
    
	if(m_pVideoFrameRGB!=NULL)
	{
		av_free(m_pVideoFrameRGB);
		m_pVideoFrameRGB = NULL;
	}
    
	// Free the YUV frame
	if(m_pVideoFrame!=NULL)
	{
		av_free(m_pVideoFrame);
		m_pVideoFrame = NULL;
	}
    
	if(m_pAudioFrame!=NULL)
	{
		av_free(m_pAudioFrame);
		m_pAudioFrame = NULL;
	}
    
    
	if(m_pSwScalingContext!=NULL)
	{
		sws_freeContext(m_pSwScalingContext);
		m_pSwScalingContext = NULL;
	}
    
    if(m_pSwResampleContext !=NULL)
	{
        swr_free(&m_pSwResampleContext);
		m_pSwResampleContext = NULL;
	}
    
	// Close the codecs
	if(m_pVideoCodecContext!=NULL)
	{
		avcodec_close(m_pVideoCodecContext);
		m_pVideoCodecContext = NULL;
	}
	if(m_pAudioCodecContext!=NULL)
	{
		avcodec_close(m_pAudioCodecContext);
		m_pAudioCodecContext = NULL;
	}
    
	// Close the file
	if(m_pFormatContext!=NULL)
	{
		//avformat_close_input(&m_pFormatContext);
		avformat_free_context(m_pFormatContext);	// this line should free all the associated mem with file, todo seriously check on lost mem blocks
		m_pFormatContext = NULL;
	}

}
void ofxFFMPEGVideoPlayer::update()
{
    
    isFrameDecoded = false;
    static bool bIsSeekable = true;
    
    if(isImage())	// no update needed for already decoded image
        return;
    
    // make sure always to decode 0 frame first, even if the first delta time would suggest differently
    if(m_dCurrentTimeInMs<0)
        m_dTargetTimeInMs = 0;
    
    long lTargetFrame = m_lCurrentFrameNumber;//calculateFrameNumberFromTime(m_dTargetTimeInMs);
    
    
    if(bIsSeekable)
    {
        //bIsSeekable = seekFrame(lTargetFrame);
        isFrameDecoded = decodeFrame();
        if(isFrameDecoded)
        {
            if(m_Mutex.tryLock())
            {
                if (m_AVData.m_VideoData.m_pData == NULL) {
                    m_Mutex.unlock();
                    return;
                }
                m_VideoTex.loadData(m_AVData.m_VideoData.m_pData, getWidth(), getHeight(), GL_RGB);
                //m_VideoPbo.loadData(m_AVData.m_VideoData.m_pData);
                //m_VideoPbo.updateTexture();
                
                m_bHavePixelsChanged = true;
                newFrame = true;
                
                m_Mutex.unlock();
            }
        }
    }
    
//    if(!bIsSeekable)	// just forward decoding, this is a huge performance issue when playing backwards
//    {
//        /*if(lTargetFrame<=0)
//         {
//         seekFrame(0);
//         m_lCurrentFrameNumber = m_dCurrentTimeInMs = 0;
//         }*/
//        
//        if( m_iState == ePlaying)
//        {
//            isFrameDecoded = decodeFrame();
//            m_dTargetTimeInMs = (float)(lTargetFrame) * 1.0 / m_dFps * 1000.0;
//            
//        }
//    }
    
    //m_lCurrentFrameNumber++;// = lTargetFrame;
    //m_dCurrentTimeInMs = m_dTargetTimeInMs;

    
    
//    if(m_Mutex.tryLock())
//	{
//        if (newFrame) {
//            if (!m_VideoTex.isAllocated()) {
//                m_VideoTex.allocate(getWidth(), getHeight(), GL_RGB);
//            }
//            
//            if (m_AVData.m_VideoData.m_pData != NULL)
//                m_VideoTex.loadData(m_AVData.m_VideoData.m_pData, getWidth(), getHeight(), GL_RGB);
//            
//            m_bHavePixelsChanged = true;
//            newFrame = false;
//        }
//        m_Mutex.unlock();
//	}
//    

}
void ofxFFMPEGVideoPlayer::play()
{
	if(!isImage())
	{
		m_iState = ePlaying;
	}
    
}

void ofxFFMPEGVideoPlayer::pause()
{
	m_iState = ePaused;
}

void ofxFFMPEGVideoPlayer::stop()
{
 	m_lCurrentFrameNumber = -1;	// set to invalid, as it is not decoded yet
	m_dTargetTimeInMs = 0;
	m_iState = eStopped;
	if(m_bIsFileOpen)
		seekFrame(0);	// so unseekable files get reset too
   
}

bool ofxFFMPEGVideoPlayer::isFrameNew()
{
    //update();
//	long lTargetFrame = calculateFrameNumberFromTime(m_dTargetTimeInMs);
//	return (lTargetFrame != m_lCurrentFrameNumber);
    return m_bHavePixelsChanged;
}
unsigned char * ofxFFMPEGVideoPlayer::getPixels()
{
    return m_AVData.m_VideoData.m_pData;
}

ofTexture * ofxFFMPEGVideoPlayer::getTexture()
{
    m_bHavePixelsChanged = false;
    return &m_VideoTex;
}
// if your videoplayer needs to implement seperate texture and pixel returns for performance, implement this function to return a texture instead of a pixel array. see iPhoneVideoGrabber for reference

float ofxFFMPEGVideoPlayer::getWidth()
{
    return m_AVData.m_VideoData.m_iWidth;
}

float ofxFFMPEGVideoPlayer::getHeight()
{
    return m_AVData.m_VideoData.m_iHeight;
}

bool ofxFFMPEGVideoPlayer::isPaused()
{
    
}

bool ofxFFMPEGVideoPlayer::isLoaded()
{
    
}

bool ofxFFMPEGVideoPlayer::isPlaying()
{
    
}

bool ofxFFMPEGVideoPlayer::setPixelFormat(ofPixelFormat pixelFormat)
{
    
}

ofPixelFormat ofxFFMPEGVideoPlayer::getPixelFormat()
{
    
}

void ofxFFMPEGVideoPlayer::setPosition(float pct)
{
    m_dTargetTimeInMs = pct;
}

void ofxFFMPEGVideoPlayer::setFrame(int frame)
{
    
    seekFrame(frame);
    //m_dTargetTimeInMs = ((float)(frame) * 1.0 / m_dFps * 1000.0);
}

double ofxFFMPEGVideoPlayer::getFrameRate()
{
    double fps = r2d(m_pFormatContext->streams[m_iVideoStream]->r_frame_rate);
    
//#if LIBAVFORMAT_BUILD >= CALC_FFMPEG_VERSION(52, 111, 0)
//    if (fps < eps_zero)
//    {
//        fps = r2d(ic->streams[video_stream]->avg_frame_rate);
//    }
//#endif
    
    if (fps < EPS)
    {
        fps = 1.0 / r2d(m_pFormatContext->streams[m_iVideoStream]->codec->time_base);
    }
    
    return fps;
}

float ofxFFMPEGVideoPlayer::getDurationSec()
{
    double sec = (double)m_pFormatContext->duration / (double)AV_TIME_BASE;
    
    if (sec < EPS)
    {
        sec = (double)m_pFormatContext->streams[m_iVideoStream]->duration * r2d(m_pFormatContext->streams[m_iVideoStream]->time_base);
    }
    
    if (sec < EPS)
    {
        sec = (double)m_pFormatContext->streams[m_iVideoStream]->duration * r2d(m_pFormatContext->streams[m_iVideoStream]->time_base);
    }
    
    return sec;

}

int ofxFFMPEGVideoPlayer::getTotalNumFrames()
{
    if (m_pFormatContext == NULL) {
        return -1;
    }
    
    int64_t nbf = m_pFormatContext->streams[m_iVideoStream]->nb_frames;
    
    if (nbf == 0)
    {
        nbf = (int64_t)floor(getDurationSec() * getFrameRate() + 0.5);
    }
    return nbf;

}


/*-------------------------------------------------*/
/*-------------- FFMPEG Implementation ------------*/
/*-------------------------------------------------*/

double ofxFFMPEGVideoPlayer::dts_to_sec(int64_t dts)
{
    return (double)(dts - m_pFormatContext->streams[m_iVideoStream]->start_time) * r2d(m_pFormatContext->streams[m_iVideoStream]->time_base);
}

int64_t ofxFFMPEGVideoPlayer::dts_to_frame_number(int64_t dts)
{
    double sec = dts_to_sec(dts);
    return (int64_t)(getFrameRate() * sec + 0.5);
}


bool ofxFFMPEGVideoPlayer::seekFrame(long lFrameNumber)
{
    
//    int frame_delta = lFrameNumber - m_pFormatContext->streams[0]-> frame_id;
//    if (frame_delta < 0 || frame_delta > 5)
//        av_seek_frame(av->fmt_ctx, av->video_stream_id,
//                      frame, AVSEEK_FLAG_BACKWARD);
//    while (av->frame_id != frame)
//        AV_read_frame(av);
//    
//    ts = ffmpeg.seekTo / ffmpeg.pFormatCtx->streams[ffmpeg.audioStream]->time_base.num * ffmpeg.pFormatCtx->streams[ffmpeg.audioStream]->time_base.den * ffmpeg.pFormatCtx->duration / AV_TIME_BASE;


    lFrameNumber = std::min((int64_t)lFrameNumber, (int64_t)getTotalNumFrames());
    int delta = 16;
    
    // if we have not grabbed a single frame before first seek, let's read the first frame
    // and get some valuable information during the process
    if( m_lCueInFrameNumber < 0 && getTotalNumFrames() > 1 )
        decodeFrame();
    
    for(;;)
    {
        int64_t _frame_number_temp = std::max((int64_t)(lFrameNumber - delta), (int64_t)0);
        double sec = (double)_frame_number_temp / getFrameRate();
        int64_t time_stamp = m_pFormatContext->streams[m_iVideoStream]->start_time;
        double  time_base  = r2d(m_pFormatContext->streams[m_iVideoStream]->time_base);
        time_stamp += (int64_t)(sec / time_base + 0.5);
        if (getTotalNumFrames() > 1) av_seek_frame(m_pFormatContext, m_iVideoStream, time_stamp, AVSEEK_FLAG_BACKWARD);
        avcodec_flush_buffers(m_pFormatContext->streams[m_iVideoStream]->codec);
        if( lFrameNumber > 0 )
        {
            decodeFrame();
            
            if( lFrameNumber > 1 )
            {
                m_lCurrentFrameNumber = dts_to_frame_number(m_AVData.m_VideoData.m_lDts) - m_lCueInFrameNumber;
                printf("_frame_number = %d, frame_number = %d, delta = %d\n", (int)lFrameNumber, (int)m_lCurrentFrameNumber, delta);
                
                if( m_lCurrentFrameNumber < 0 || m_lCurrentFrameNumber > lFrameNumber-1 )
                {
                    if( _frame_number_temp == 0 || delta >= INT_MAX/4 )
                        break;
                    delta = delta < 16 ? delta*2 : delta*3/2;
                    continue;
                }
                while( m_lCurrentFrameNumber < lFrameNumber-1 )
                {
                    if(!decodeFrame())
                        break;
                }
                m_lCurrentFrameNumber++;
                break;
            }
            else
            {
                m_lCurrentFrameNumber = 1;
                break;
            }
        }
        else
        {
            m_lCurrentFrameNumber = 0;
            break;
        }
    }


    
    
//	int iDirectionFlag = 0;
//	if(m_iDirection == eBackward)
//		iDirectionFlag = AVSEEK_FLAG_BACKWARD;
//	
//	int iStream = m_iVideoStream;
//	if(iStream<0)						// we just have an audio stream so seek in this stream
//		iStream = m_iAudioStream;
//    
//	if(iStream>=0)
//	{
//        
////        //SEEK
////        if (avformat_seek_file(m_pFormatContext, iStream, INT64_MIN, lFrameNumber, INT64_MAX, 0) < 0) {
////            av_log(NULL, AV_LOG_ERROR, "ERROR av_seek_frame: %ld\n", lFrameNumber);
////        } else {
////            av_log(NULL, AV_LOG_ERROR, "SUCCEEDED av_seek_frame: %ld newPos:%lld\n", lFrameNumber, m_pFormatContext->pb->pos);
////            avcodec_flush_buffers(m_pFormatContext->streams[iStream]->codec);
////        }
//        
//        
//        
//        if (av_seek_frame(m_pFormatContext, iStream, lFrameNumber, AVSEEK_FLAG_BACKWARD /*AVSEEK_FLAG_FRAME*/ ) < 0)
//		//if(avformat_seek_file(m_pFormatContext, iStream, lFrameNumber, lFrameNumber, lFrameNumber, AVSEEK_FLAG_FRAME | AVSEEK_FLAG_ANY | AVSEEK_FLAG_BACKWARD) < 0)
//        //if (avformat_seek_file(m_pFormatContext,iStream,0,lFrameNumber,lFrameNumber,AVSEEK_FLAG_FRAME | AVSEEK_FLAG_ANY) < 0)
//		{
//			return false;
//		}
//		if( m_pVideoCodecContext != NULL)
//			avcodec_flush_buffers(m_pVideoCodecContext);
//		if( m_pAudioCodecContext != NULL)
//			avcodec_flush_buffers(m_pAudioCodecContext);
//        
//		return true;
//	}
	return false;
    
}

bool ofxFFMPEGVideoPlayer::seekTime(double dTimeInMs)
{
	int iDirectionFlag = 0;
	if(m_iDirection == eBackward)
		iDirectionFlag = AVSEEK_FLAG_BACKWARD;
	
	int iStream = m_iVideoStream;
	if(iStream<0)						// we just have an audio stream so seek in this stream
		iStream = m_iAudioStream;
    
//	if(iStream>=0)
//	{
//		if(avformat_seek_file(m_pFormatContext, -1, dTimeInMs*1000-2, dTimeInMs*1000, dTimeInMs*1000+2, AVSEEK_FLAG_ANY | iDirectionFlag) < 0)
//		{
//			return false;
//		}
//		if( m_pVideoCodecContext != NULL)
//			avcodec_flush_buffers(m_pVideoCodecContext);
//		if( m_pAudioCodecContext != NULL)
//			avcodec_flush_buffers(m_pAudioCodecContext);
//        
//		return true;
//	}
//	return false;
    
    long frameNumber;
	if(iStream>=0)
	{
        frameNumber = av_rescale(dTimeInMs,m_pFormatContext->streams[iStream]->time_base.den,m_pFormatContext->streams[iStream]->time_base.num);
        frameNumber/=1000;
	}
	return seekFrame(frameNumber);
}


bool ofxFFMPEGVideoPlayer::openVideoStream()
{
	// Get a pointer to the codec context for the video stream
	m_pVideoCodecContext = m_pFormatContext->streams[m_iVideoStream]->codec;
    
	// Find the decoder for the video stream
	AVCodec* pCodec = avcodec_find_decoder(m_pVideoCodecContext->codec_id);	// guess this is deleted with the avcodec_close, that's what the docs say
	if(pCodec==NULL)
		return false; // Codec not found
    
	// Open codec
	AVDictionary* options;
	if(avcodec_open2(m_pVideoCodecContext, pCodec, NULL)<0)
		return false; // Could not open codec
    
	// Allocate video frame
	m_pVideoFrame = avcodec_alloc_frame();
	if(m_pVideoFrame==NULL)
		return false;
    
	// Allocate an AVFrame structure
	m_pVideoFrameRGB=avcodec_alloc_frame();
	if(m_pVideoFrameRGB==NULL)
		return false;
    
	retrieveVideoInfo();
    
	// Determine required buffer size and allocate buffer
	m_pVideoBuffer = (uint8_t*)(av_malloc(avpicture_get_size( PIX_FMT_RGB24, getWidth(), getHeight())));
    
    m_pVideoBufferYUV = (uint8_t*)(av_malloc(avpicture_get_size( PIX_FMT_YUV420P, getWidth(), getHeight())));

    avpicture_fill((AVPicture*)m_pVideoFrame, m_pVideoBufferYUV, PIX_FMT_YUV420P, getWidth(), getHeight());
    
	// Assign appropriate parts of buffer to image planes in pFrameRGB
	avpicture_fill((AVPicture*)m_pVideoFrameRGB, m_pVideoBuffer, PIX_FMT_RGB24, getWidth(), getHeight());
    
	//Initialize Context
	m_pSwScalingContext = sws_getContext(getWidth(), getHeight(), m_pVideoCodecContext->pix_fmt, getWidth(), getHeight(), PIX_FMT_RGB24, SWS_BICUBIC, NULL, NULL, NULL);
    
	return true;
}

bool ofxFFMPEGVideoPlayer::openAudioStream()
{
	// Get a pointer to the codec context for the video stream
	m_pAudioCodecContext = m_pFormatContext->streams[m_iAudioStream]->codec;
    
	// Find the decoder for the video stream
	AVCodec* pCodec = avcodec_find_decoder(m_pAudioCodecContext->codec_id);	// guess this is deleted with the avcodec_close, that's what the docs say
	if(pCodec==NULL)
		return false; // Codec not found
    
	// Open codec
	AVDictionary* options;
	if(avcodec_open2(m_pAudioCodecContext, pCodec, NULL)<0)
		return false; // Could not open codec
    
	// Allocate video frame
	m_pAudioFrame = avcodec_alloc_frame();
    
	retrieveAudioInfo();
    
    m_AudioOutChannelLayout = AV_CH_LAYOUT_STEREO;
    m_AudioOutSampleFormat  = AV_SAMPLE_FMT_FLT; //AV_SAMPLE_FMT_FLT; //AV_SAMPLE_FMT_S16;
    m_AudioOutSampleRate    = getAudioSampleRate();
    
    m_pSwResampleContext = swr_alloc_set_opts(NULL,
                                                m_AudioOutChannelLayout,
                                                m_AudioOutSampleFormat,
                                                m_AudioOutSampleRate,
                                                av_get_default_channel_layout(m_pAudioCodecContext->channels),
                                                m_pAudioCodecContext->sample_fmt,
                                                m_pAudioCodecContext->sample_rate,
                                                0,
                                                NULL);

    if (m_pSwResampleContext == NULL)
    {
        printf("Couldn't create the SwrContext");
        return false;
    }
    if (swr_init(m_pSwResampleContext) != 0)
    {
        printf("Couldn't initialize the SwrContext");
        return false;
    }
    
//    wav.sampleRate = m_AudioOutSampleRate;
//    wav.sampleSize = av_get_bytes_per_sample(m_AudioOutSampleFormat);
//    printf("Bytes Per Sample: %d",  wav.sampleSize);
//    wav.channels = av_get_channel_layout_nb_channels(m_AudioOutChannelLayout);
    
	return true;
}

void ofxFFMPEGVideoPlayer::retrieveFileInfo()
{
	m_iBitrate = m_pFormatContext->bit_rate / 1000.0;
    
	int iStream = m_iVideoStream;
	if(iStream<0)						// we just have an audio stream so seek in this stream
		iStream = m_iAudioStream;
    
	if(iStream>=0)
	{
		m_dDurationInMs = m_pFormatContext->duration * 1000.0 / (float)AV_TIME_BASE;
		if(m_dDurationInMs/1000.0 < EPS)
			m_dDurationInMs =  m_pFormatContext->streams[iStream]->duration * r2d( m_pFormatContext->streams[iStream]->time_base) * 1000.0;
		m_dFps = r2d(m_pFormatContext->streams[iStream]->r_frame_rate);
		if(m_dFps < EPS)
			m_dFps = r2d(m_pFormatContext->streams[iStream]->avg_frame_rate);
        
		m_lDurationInFrames = m_pFormatContext->streams[iStream]->nb_frames;		// for some codec this return wrong numbers so calc from time
		if(m_lDurationInFrames == 0)
			m_lDurationInFrames = (int)(( m_pFormatContext->duration / (double)AV_TIME_BASE ) * m_pVideoCodecContext->time_base.den + 0.5);//calculateFrameNumberFromTime(m_dDurationInMs);
	}
}

void ofxFFMPEGVideoPlayer::retrieveVideoInfo()
{
	m_strVideoCodecName = std::string(m_pVideoCodecContext->codec->long_name);
	m_AVData.m_VideoData.m_iWidth = m_pVideoCodecContext->width;
	m_AVData.m_VideoData.m_iHeight = m_pVideoCodecContext->height;
}

void ofxFFMPEGVideoPlayer::retrieveAudioInfo()
{
	m_strAudioCodecName = std::string(m_pAudioCodecContext->codec->long_name);
	m_AVData.m_AudioData.m_iSampleRate = m_pAudioCodecContext->sample_rate;
	m_AVData.m_AudioData.m_iChannels = m_pAudioCodecContext->channels;
}

void ofxFFMPEGVideoPlayer::dumpFFmpegInfo()
{
	std::cout << "_2RealFFmpegWrapper 0.1" << std::endl;
	std::cout << "http://www.cadet.at" << std::endl << std::endl;
	std::cout << "Please regard the license of the here wrapped FFmpeg library, LGPL or GPL_V3 depending on enabled codecs and configuration" << std::endl;
	std::cout << "FFmpeg license: " << avformat_license() << std::endl;
	std::cout << "AVCodec version " << LIBAVFORMAT_VERSION_MAJOR << "." << LIBAVFORMAT_VERSION_MINOR << "." << LIBAVFORMAT_VERSION_MICRO << std::endl;
	std::cout << "AVFormat configuration: " << avformat_configuration() << std::endl << std::endl;
}

AVPacket* ofxFFMPEGVideoPlayer::fetchAVPacket()
{
	AVPacket *pAVPacket = NULL;
    
	pAVPacket = new AVPacket();
	if(av_read_frame(m_pFormatContext, pAVPacket)>=0)
		return pAVPacket;
	else
		return NULL;
}

bool ofxFFMPEGVideoPlayer::decodeFrame()
{
	bool bRet = false;
	
	for(int i=0; i<m_pFormatContext->nb_streams; i++)
	{
		AVPacket* pAVPacket = fetchAVPacket();
		
		if(pAVPacket!=NULL)
		{
			// Is this a packet from the video stream?
			if(pAVPacket->stream_index == m_iVideoStream)
			{
				bRet = false;
				bRet = decodeVideoFrame(pAVPacket);
			}
			else if(pAVPacket->stream_index == m_iAudioStream)
			{
				bRet = false;
				bRet = decodeAudioFrame(pAVPacket);
			}
            
			av_free_packet(pAVPacket);
            
			if(!bRet)
				return false;
		}
	}
	
	if(!bRet)
		printf("ficke");
	return bRet;
}

bool ofxFFMPEGVideoPlayer::decodeVideoFrame(AVPacket* pAVPacket)
{
	int isFrameDecoded=0;
    int bytes = 0;

    // Decode video frame
    while(!isFrameDecoded){
        bytes = avcodec_decode_video2(m_pVideoCodecContext, m_pVideoFrame, &isFrameDecoded, pAVPacket);
        if (bytes < 0) {
            return false;
        }
    }

	// Did we get a video frame?
	if(!isFrameDecoded)
        return false;
    
    m_Mutex.lock();

    //Convert YUV->RGB
    sws_scale(m_pSwScalingContext, m_pVideoFrame->data, m_pVideoFrame->linesize, 0, getHeight(), m_pVideoFrameRGB->data, m_pVideoFrameRGB->linesize);
    m_AVData.m_VideoData.m_pData =  m_pVideoFrameRGB->data[0];
    m_AVData.m_VideoData.m_lPts = m_pVideoFrame->pkt_pts;
    m_AVData.m_VideoData.m_lDts = m_pVideoFrame->pkt_dts;
    if(m_AVData.m_VideoData.m_lPts == AV_NOPTS_VALUE)
        m_AVData.m_VideoData.m_lPts = 0;
    if(m_AVData.m_VideoData.m_lDts == AV_NOPTS_VALUE)
        m_AVData.m_VideoData.m_lDts = 0;
    
    printf("video frame %ld\n", m_AVData.m_VideoData.m_lPts);
    
    m_lCurrentFrameNumber++;
    if( m_lCueInFrameNumber < 0 )
    {
        printf("First frame %ld\n", dts_to_frame_number(m_AVData.m_VideoData.m_lPts));

        m_lCueInFrameNumber = dts_to_frame_number(m_AVData.m_VideoData.m_lPts);
    }
    
    m_Mutex.unlock();
        
		
    return true;

}

void ofxFFMPEGVideoPlayer::saveWav(const std::string& filename, const Wav& wav)
{
    // see [url="https://ccrma.stanford.edu/courses/422/projects/WaveFormat/"]https://ccrma.stanford.edu/courses/422/projects/WaveFormat/[/url]
    std::ofstream file(filename.c_str(), std::ios::binary);
    
    char chunkId[] = {'R', 'I', 'F', 'F'};
    unsigned int chunkSize = 36 + wav.data.size();
    char format[] = {'W', 'A', 'V', 'E'};
    
    // Write the first header
    file.write(chunkId, 4);
    file.write((char*)&chunkSize, 4);
    file.write(format, 4);
    
    chunkId[0] = 'f'; chunkId[1] = 'm'; chunkId[2] = 't'; chunkId[3] = ' ';
    chunkSize = 16;
    short audioFormat = 1;
    int byteRate = wav.channels * wav.sampleRate * wav.sampleSize;
    short blockAlign = wav.channels * wav.sampleSize;
    short bitsPerSample = wav.sampleSize * 8;
    
    // Write the second header
    file.write(chunkId, 4);
    file.write((char*)&chunkSize, 4);
    file.write((char*)&audioFormat, 2);
    file.write((char*)&wav.channels, 2);
    file.write((char*)&wav.sampleRate, 4);
    file.write((char*)&byteRate, 4);
    file.write((char*)&blockAlign, 2);
    file.write((char*)&bitsPerSample, 2);
    
    chunkId[0] = 'd'; chunkId[1] = 'a'; chunkId[2] = 't'; chunkId[3] = 'a';
    chunkSize = wav.data.size();
    
    // Write the third header and the actual data
    file.write(chunkId, 4);
    file.write((char*)&chunkSize, 4);
    file.write((char*)&wav.data[0], wav.data.size());
}


bool ofxFFMPEGVideoPlayer::decodeAudioFrame(AVPacket* pAVPacket)
{
	int isFrameDecoded=0;
    
    m_Mutex.lock();

	if(avcodec_decode_audio4(m_pAudioCodecContext, m_pAudioFrame, &isFrameDecoded, pAVPacket)<0)
	{
		m_AVData.m_AudioData.m_pData = NULL;
		return false;
	}
    
    int dst_nb_samples = av_rescale_rnd(m_pAudioFrame->nb_samples, m_AudioOutSampleRate, m_AVData.m_AudioData.m_iSampleRate, AV_ROUND_UP);
    int dst_nb_channels = av_get_channel_layout_nb_channels(m_AudioOutChannelLayout);
    av_samples_alloc(&m_AVData.m_AudioData.m_pData, NULL, dst_nb_channels, dst_nb_samples, m_AudioOutSampleFormat, 0);
    
    int numSamplesOut = swr_convert(m_pSwResampleContext, &m_AVData.m_AudioData.m_pData, m_AudioOutSampleRate, (const unsigned char**)m_pAudioFrame->extended_data, m_pAudioFrame->nb_samples);
    if (numSamplesOut < 0)
    {
        printf("Audio resampling failed...\n");
        return;
    }
    
    if (numSamplesOut == 0)
    {
        // it consumed all the samples and is storing them in an internal buffer
        return;
    }

	//m_AVData.m_AudioData.m_pData = m_pAudioFrame->data[0];
	m_AVData.m_AudioData.m_lSizeInBytes = av_samples_get_buffer_size(NULL, m_pAudioCodecContext->channels, m_pAudioFrame->nb_samples, m_pAudioCodecContext->sample_fmt, 1);	// 1 stands for don't align size
	m_AVData.m_AudioData.m_lPts = m_pAudioFrame->pkt_pts;
	m_AVData.m_AudioData.m_lDts = m_pAudioFrame->pkt_dts;
	if(m_AVData.m_AudioData.m_lPts == AV_NOPTS_VALUE)
		m_AVData.m_AudioData.m_lPts = 0;
	if(m_AVData.m_AudioData.m_lDts == AV_NOPTS_VALUE)
		m_AVData.m_AudioData.m_lDts = 0;
    m_AVData.m_AudioData.m_iSamplesCount = numSamplesOut;
	printf("audio frame %d\n", m_AVData.m_AudioData.m_lPts);
    printf("audio size %d bytes\n", m_AVData.m_AudioData.m_lSizeInBytes);
    
//    for (int i = 0; i < numSamplesOut * wav.sampleSize * wav.channels; ++i)
//    {
//        wav.data.push_back(m_AVData.m_AudioData.m_pData[i]);
//    }
    
    m_Mutex.unlock();
    
	return true;
}

bool ofxFFMPEGVideoPlayer::decodeImage()
{
	int isFrameDecoded=-1;
    
	AVPacket packet;
	// alloc img buffer
	FILE *imgFile = fopen(m_strFileName.c_str(),"rb");
	fseek(imgFile,0,SEEK_END);
	long imgFileSize = ftell(imgFile);
	fseek(imgFile,0,SEEK_SET);
	void *imgBuffer = malloc(imgFileSize);
	fread(imgBuffer,1,imgFileSize,imgFile);
	fclose(imgFile);
	packet.data = (uint8_t*)imgBuffer;
	packet.size = imgFileSize;
	av_init_packet(&packet);
	//decode image
	avcodec_decode_video2(m_pVideoCodecContext, m_pVideoFrame, &isFrameDecoded, &packet);
    
	if(isFrameDecoded)	// Did we get a video frame?
	{
		//Convert YUV->RGB
		sws_scale(m_pSwScalingContext, m_pVideoFrame->data, m_pVideoFrame->linesize, 0, getHeight(), m_pVideoFrameRGB->data, m_pVideoFrameRGB->linesize);
		av_free_packet(&packet);
		free(imgBuffer);			// we have to free this buffer separately don't ask me why, otherwise leak
		return true;
	}
	else
	{
		free(imgBuffer);
	    av_free_packet(&packet);
		return false;
	}
}

AVData& ofxFFMPEGVideoPlayer::getAVData()
{
	update();
	return m_AVData;
}

VideoData& ofxFFMPEGVideoPlayer::getVideoData()
{
	//update();
	return m_AVData.m_VideoData;
}

AudioData& ofxFFMPEGVideoPlayer::getAudioData()
{    
    //update();
	return m_AVData.m_AudioData;
}

std::string ofxFFMPEGVideoPlayer::getVideoCodecName()
{
	return m_strVideoCodecName;
}

std::string ofxFFMPEGVideoPlayer::getAudioCodecName()
{
	return m_strAudioCodecName;
}


int ofxFFMPEGVideoPlayer::getAudioChannels()
{
	return m_AVData.m_AudioData.m_iChannels;
}

int ofxFFMPEGVideoPlayer::getAudioSampleRate()
{
	return m_AVData.m_AudioData.m_iSampleRate;
}


long ofxFFMPEGVideoPlayer::calculateFrameNumberFromTime(long lTime)
{
	long lTargetFrame = floor((double)lTime/1000.0 * m_dFps );	//the 0.5 is taken from the opencv player, this might be useful for floating point rounding problems to be on the safe side not to miss one frame
	return lTargetFrame;
}

bool ofxFFMPEGVideoPlayer::hasVideo()
{
	return m_iVideoStream >= 0;
}

bool ofxFFMPEGVideoPlayer::hasAudio()
{
	return m_iAudioStream >= 0;
}


bool ofxFFMPEGVideoPlayer::isImage()
{
	return m_iBitrate<=0 && m_AVData.m_AudioData.m_iSampleRate<=0;
}

// helper function as taken from OpenCV ffmpeg reader
double ofxFFMPEGVideoPlayer::r2d(AVRational r)
{
    return r.num == 0 || r.den == 0 ? 0. : (double)r.num / (double)r.den;
}

