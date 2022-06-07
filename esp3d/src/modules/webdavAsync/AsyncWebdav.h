#ifndef __ESPASYNCWEBDAV_H
#define __ESPASYNCWEBDAV_H

#include "../../include/esp3d_config.h"


#include <Arduino.h>
#include <FS.h>
#include <ESPAsyncWebServer.h>

enum DavResourceType { DAV_RESOURCE_NONE, DAV_RESOURCE_FILE, DAV_RESOURCE_DIR };
enum DavDepthType { DAV_DEPTH_NONE, DAV_DEPTH_CHILD, DAV_DEPTH_ALL };


class AsyncWebdav: public AsyncWebHandler {
    using FS = fs::FS;
    public:
        AsyncWebdav(const String& url, FS &fs);

        virtual bool canHandle(AsyncWebServerRequest *request) override final;
        virtual void handleRequest(AsyncWebServerRequest *request) override final;
        virtual void handleBody(AsyncWebServerRequest *request, unsigned char *data, size_t len, size_t index, size_t total) override final;
    public:
        const char* url() const { return _url.c_str(); }
        void setReject(String rejectMessage) { _isReject = true; _rejectMessage = rejectMessage; }
        void clearReject() { _isReject = false; _rejectMessage = "" ; }
    private:
        String _url;
        FS _fs;
        String _rejectMessage;
        bool _isReject = false;

        void handlePropfind(const String& path, DavResourceType resource, AsyncWebServerRequest * request);
        void handleGet(const String& path, DavResourceType resource, AsyncWebServerRequest * request);
        void handlePut(const String& path, DavResourceType resource, AsyncWebServerRequest * request, unsigned char *data, size_t len, size_t index, size_t total);
        void handleLock(const String& path, DavResourceType resource, AsyncWebServerRequest * request);
        void handleUnlock(const String& path, DavResourceType resource, AsyncWebServerRequest * request);
        void handleMkcol(const String& path, DavResourceType resource, AsyncWebServerRequest * request);
        void handleMove(const String& path, DavResourceType resource, AsyncWebServerRequest * request);
        void handleDelete(const String& path, DavResourceType resource, AsyncWebServerRequest * request);
        void handleHead(DavResourceType resource, AsyncWebServerRequest * request);
        void handleNotFound(AsyncWebServerRequest * request);
        void handleReject(AsyncWebServerRequest *request);

        
        void sendPropResponse(AsyncResponseStream *response, boolean recursing, File *curFile);
        String date2date(time_t date);
        String urlToUri(String url);

};

#endif // __ESPASYNCWEBDAV_H
