/*
sd_native_esp8266.cpp - ESP3D sd support class

  Copyright (c) 2014 Luc Lebosse. All rights reserved.

  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with This code; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "../../../include/esp3d_config.h"
#if defined (ARDUINO_ARCH_ESP32) && defined(SD_DEVICE)
#if (SD_DEVICE == ESP_SDFAT2)
#include "../esp_sd.h"
#include "../../../core/genLinkedList.h"
#include "../../../core/settings_esp3d.h"
#include <SdFat.h>
#include <sdios.h>

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig((ESP_SD_CS_PIN == -1)?SS:ESP_SD_CS_PIN, DEDICATED_SPI)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig((ESP_SD_CS_PIN == -1)?SS:ESP_SD_CS_PIN, SHARED_SPI)
#endif  // HAS_SDIO_CLASS

extern FsFile tSDFile_handle[ESP_MAX_SD_OPENHANDLE];

//Max Freq Working
#define FREQMZ 40
SdFs SD;

void dateTime (uint16_t* date, uint16_t* dtime)
{
    struct tm  tmstruct;
    time_t now;
    time (&now);
    localtime_r (&now, &tmstruct);
    *date = FAT_DATE ( (tmstruct.tm_year) + 1900, ( tmstruct.tm_mon) + 1, tmstruct.tm_mday);
    *dtime = FAT_TIME (tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
}

time_t getDateTimeFile(FsFile & filehandle)
{
    static time_t dt = 0;
#ifdef SD_TIMESTAMP_FEATURE
    struct tm timefile;
    uint16_t date;
    uint16_t time;

    if(filehandle) {
        if (filehandle.getModifyDateTime(&date, &time)) {
            timefile.tm_year = FS_YEAR(date) - 1900;
            timefile.tm_mon = FS_MONTH(date) - 1;
            timefile.tm_mday = FS_DAY(date);
            timefile.tm_hour = FS_HOUR(time);
            timefile.tm_min = FS_MINUTE(time);
            timefile.tm_sec = FS_SECOND(time);
            timefile.tm_isdst = -1;
            dt =  mktime(&timefile);
            if (dt == -1) {
                log_esp3d("mktime failed");
            }
        } else {
            log_esp3d("stat file failed");
        }
    } else {
        log_esp3d("check file for stat failed");
    }
#endif //SD_TIMESTAMP_FEATURE
    return dt;
}


uint8_t ESP_SD::getState(bool refresh)
{
#if defined(ESP_SD_DETECT_PIN) && ESP_SD_DETECT_PIN != -1
    //no need to go further if SD detect is not correct
    if (!((digitalRead (ESP_SD_DETECT_PIN) == ESP_SD_DETECT_VALUE) ? true : false)) {
        _state = ESP_SDCARD_NOT_PRESENT;
        return _state;
    }
#endif  //ESP_SD_DETECT_PIN
    //if busy doing something return state
    if (!((_state == ESP_SDCARD_NOT_PRESENT) || _state == ESP_SDCARD_IDLE)) {
        return _state;
    }
    if (!refresh) {
        return _state;  //to avoid refresh=true + busy to reset SD and waste time
    } else {
        _sizechanged = true;
    }
    //SD is idle or not detected, let see if still the case
    _state = ESP_SDCARD_NOT_PRESENT;
    bool isactive = accessSD();
    log_esp3d("Spi : CS: %d,  Miso: %d, Mosi: %d, SCK: %d",ESP_SD_CS_PIN!=-1?ESP_SD_CS_PIN:SS, ESP_SD_MISO_PIN!=-1?ESP_SD_MISO_PIN:MISO, ESP_SD_MOSI_PIN!=-1?ESP_SD_MOSI_PIN:MOSI, ESP_SD_SCK_PIN!=-1?ESP_SD_SCK_PIN:SCK);
    //refresh content if card was removed
    if (SD.begin((ESP_SD_CS_PIN == -1)?SS:ESP_SD_CS_PIN, SD_SCK_MHZ(FREQMZ/1))) {
        csd_t m_csd;
        if (SD.card()->readCSD(&m_csd) && sdCardCapacity(&m_csd) > 0 ) {
            _state = ESP_SDCARD_IDLE;
        } else {
            log_esp3d("SD.card()->readCSD faild.");
        }
    } else {
        log_esp3d("SD.begin faild.");
    }
    if (!isactive) {
        releaseSD();
    }
    return _state;
}

bool ESP_SD::begin()
{
#if (ESP_SD_CS_PIN != -1) || (ESP_SD_MISO_PIN != -1) || (ESP_SD_MOSI_PIN != -1) || (ESP_SD_SCK_PIN != -1)
    SPI.begin(ESP_SD_SCK_PIN, ESP_SD_MISO_PIN, ESP_SD_MOSI_PIN, ESP_SD_CS_PIN);
#endif
    _started = true;
    _state = ESP_SDCARD_NOT_PRESENT;
    _spi_speed_divider = Settings_ESP3D::read_byte(ESP_SD_SPEED_DIV);
    //sanity check
    if (_spi_speed_divider <= 0) {
        _spi_speed_divider = 1;
    }
#ifdef SD_TIMESTAMP_FEATURE
    //set callback to get time on files on SD
    SdFile::dateTimeCallback (dateTime);
#endif //SD_TIMESTAMP_FEATURE
//Setup pins
#if defined(ESP_SD_DETECT_PIN) && ESP_SD_DETECT_PIN != -1
    pinMode (ESP_SD_DETECT_PIN, INPUT);
#endif //ESP_SD_DETECT_PIN
#if SD_DEVICE_CONNECTION  == ESP_SHARED_SD
#if defined(ESP_FLAG_SHARED_SD_PIN) && ESP_FLAG_SHARED_SD_PIN != -1
    pinMode (ESP_FLAG_SHARED_SD_PIN, OUTPUT);
    digitalWrite(ESP_FLAG_SHARED_SD_PIN, !ESP_FLAG_SHARED_SD_VALUE);
#endif //ESP_FLAG_SHARED_SD_PIN
#endif //SD_DEVICE_CONNECTION  == ESP_SHARED_SD
    return _started;
}

void ESP_SD::end()
{
    _state = ESP_SDCARD_NOT_PRESENT;
    _started = false;
}

uint64_t ESP_SD::totalBytes()
{
    if (!SD.volumeBegin()) {
        return 0;
    }
    uint64_t volTotal = SD.clusterCount();
    uint8_t sectors = SD.sectorsPerCluster();
    return volTotal * sectors * 512;
}

uint64_t ESP_SD::usedBytes()
{
    if(freeBytes() >totalBytes() ) {
        _sizechanged = true;
    }
    return totalBytes() - freeBytes();
}

uint ESP_SD::maxPathLength()
{
    return 255;
}

uint64_t ESP_SD::freeBytes()
{
    static uint64_t volFree;
    if (!SD.volumeBegin()) {
        _sizechanged = true;
        return 0;
    }
    if (_sizechanged) {
        volFree = SD.freeClusterCount();
        _sizechanged = false;
    }
    uint8_t sectors = SD.sectorsPerCluster();
    return volFree * sectors * 512;
}

bool ESP_SD::rename(const char *oldpath, const char *newpath)
{
    return SD.rename(oldpath,newpath);
}

bool ESP_SD::format(ESP3DOutput * output)
{
    if (ESP_SD::getState(true) == ESP_SDCARD_IDLE) {
        uint32_t const ERASE_SIZE = 262144L;
        uint32_t cardSectorCount = 0;
        uint8_t  sectorBuffer[512];
// SdCardFactory constructs and initializes the appropriate card.
        SdCardFactory cardFactory;
// Pointer to generic SD card.
        SdCard* m_card = nullptr;
//prepare
        m_card = cardFactory.newCard(SD_CONFIG);
        if (!m_card || m_card->errorCode()) {
            if (output) {
                output->printMSG("card init failed.");
            }
            return false;
        }

        cardSectorCount = m_card->sectorCount();
        if (!cardSectorCount) {
            if (output) {
                output->printMSG("Get sector count failed.");
            }
            return false;
        }

        if (output) {
            String s = "Capacity detected :" + String(cardSectorCount*5.12e-7) + "GB";
            output->printMSG(s.c_str());
        }

        uint32_t firstBlock = 0;
        uint32_t lastBlock;
        uint16_t n = 0;
        do {
            lastBlock = firstBlock + ERASE_SIZE - 1;
            if (lastBlock >= cardSectorCount) {
                lastBlock = cardSectorCount - 1;
            }
            if (!m_card->erase(firstBlock, lastBlock)) {
                if (output) {
                    output->printMSG("erase failed");
                }
                return false;
            }

            firstBlock += ERASE_SIZE;
            if ((n++)%64 == 63) {
                Hal::wait(0);
            }
        } while (firstBlock < cardSectorCount);

        if (!m_card->readSector(0, sectorBuffer)) {
            if (output) {
                output->printMSG("readBlock");
            }
        }

        ExFatFormatter exFatFormatter;
        FatFormatter fatFormatter;

        // Format exFAT if larger than 32GB.
        bool rtn = cardSectorCount > 67108864 ?
                   exFatFormatter.format(m_card, sectorBuffer, nullptr) :
                   fatFormatter.format(m_card, sectorBuffer, nullptr);

        if (!rtn) {
            if (output) {
                output->printMSG("erase failed");
            }
            return false;
        }

        return true;
    }
    if (output) {
        output->printMSG("cannot erase");
    }
    return false;
}

ESP_SDFile ESP_SD::open(const char* path, uint8_t mode)
{
    //do some check
    if(((strcmp(path,"/") == 0) && ((mode == ESP_FILE_WRITE) || (mode == ESP_FILE_APPEND))) || (strlen(path) == 0)) {
        _sizechanged = true;
        return ESP_SDFile();
    }
    // path must start by '/'
    if (path[0] != '/') {
        return ESP_SDFile();
    }
    if (mode != ESP_FILE_READ) {
        //check container exists
        String p = path;
        p.remove(p.lastIndexOf('/') +1);
        if (!exists(p.c_str())) {
            log_esp3d("Error opening: %s", path);
            return ESP_SDFile();
        }
    }
    FsFile tmp = SD.open(path, (mode == ESP_FILE_READ)?FILE_READ:(mode == ESP_FILE_WRITE)?FILE_WRITE:FILE_WRITE);
    ESP_SDFile esptmp(&tmp, tmp.isDir(),(mode == ESP_FILE_READ)?false:true, path);
    return esptmp;
}

bool ESP_SD::exists(const char* path)
{
    bool res = false;
    //root should always be there if started
    if (strcmp(path, "/") == 0) {
        return _started;
    }
    res = SD.exists(path);
    if (!res) {
        ESP_SDFile root = ESP_SD::open(path, ESP_FILE_READ);
        if (root) {
            res = root.isDirectory();
        }
    }
    return res;
}

bool ESP_SD::remove(const char *path)
{
    _sizechanged = true;
    return SD.remove(path);
}

bool ESP_SD::mkdir(const char *path)
{
    return SD.mkdir(path);
}

bool ESP_SD::rmdir(const char *path)
{
    if (!exists(path)) {
        return false;
    }
    bool res = true;
    GenLinkedList<String > pathlist;
    String p = path;
    pathlist.push(p);
    while (pathlist.count() > 0) {
        FsFile dir = SD.open(pathlist.getLast().c_str());
        dir.rewindDirectory();
        FsFile f = dir.openNextFile();
        bool candelete = true;
        while (f) {
            if (f.isDir()) {
                candelete = false;
                String newdir;
                char tmp[255];
                f.getName(tmp,254);
                newdir = tmp;
                pathlist.push(newdir);
                f.close();
                f = FsFile();
            } else {
                char tmp[255];
                f.getName(tmp,254);
                _sizechanged = true;
                SD.remove(tmp);
                f.close();
                f = dir.openNextFile();
            }
        }
        if (candelete) {
            if (pathlist.getLast() !="/") {
                res = SD.rmdir(pathlist.getLast().c_str());
            }
            pathlist.pop();
        }
        dir.close();
    }
    p = String();
    log_esp3d("count %d", pathlist.count());
    return res;
}

void ESP_SD::closeAll()
{
    for (uint8_t i = 0; i < ESP_MAX_SD_OPENHANDLE; i++) {
        tSDFile_handle[i].close();
        tSDFile_handle[i] = FsFile();
    }
}

bool ESP_SDFile::seek(uint32_t pos, uint8_t mode)
{
    if (mode == ESP_SEEK_END) {
        return tSDFile_handle[_index].seek(-pos);    //based on SDFS comment
    }
    return tSDFile_handle[_index].seek(pos);
}

ESP_SDFile::ESP_SDFile(void* handle, bool isdir, bool iswritemode, const char * path)
{
    _isdir = isdir;
    _dirlist = "";
    _index = -1;
    _filename = "";
    _name = "";
    _lastwrite = 0 ;
    _iswritemode = iswritemode;
    _size = 0;
    if (!handle) {
        return ;
    }
    bool set =false;
    for (uint8_t i=0; (i < ESP_MAX_SD_OPENHANDLE) && !set; i++) {
        if (!tSDFile_handle[i]) {
            tSDFile_handle[i] = *((FsFile*)handle);
            //filename
            char tmp[255];
            tSDFile_handle[i].getName(tmp,254);
            _filename = path;
            //name
            _name = tmp;
            if (_name.endsWith("/")) {
                _name.remove( _name.length() - 1,1);
                _isdir = true;
            }
            if (_name[0] == '/') {
                _name.remove( 0, 1);
            }
            int pos = _name.lastIndexOf('/');
            if (pos != -1) {
                _name.remove( 0, pos+1);
            }
            if (_name.length() == 0) {
                _name = "/";
            }
            //size
            _size = tSDFile_handle[i].size();
            //time
            if (!_isdir && !iswritemode) {
                _lastwrite = getDateTimeFile(tSDFile_handle[i]);

            } else {
                //no need date time for directory
                _lastwrite = 0;
            }
            _index = i;
            //log_esp3d("Opening FsFile at index %d",_index);
            set = true;
        }
    }
}
//todo need also to add short filename
const char* ESP_SDFile::shortname() const
{
    // static char sname[13];
    // FsFile ftmp = SD.open(_filename.c_str());
    // if (ftmp) {
    //     ftmp.getSFN(sname);
    //     ftmp.close();
    //     return sname;
    // } else {
    //     return _name.c_str();
    // }
    return _name.c_str();
}

void ESP_SDFile::close()
{
    if (_index != -1) {
        //log_esp3d("Closing FsFile at index %d", _index);
        tSDFile_handle[_index].close();
        //reopen if mode = write
        //udate size + date
        if (_iswritemode && !_isdir) {
            FsFile ftmp = SD.open(_filename.c_str());
            if (ftmp) {
                _size = ftmp.size();
                _lastwrite = getDateTimeFile(ftmp);
                ftmp.close();
            }
        }
        tSDFile_handle[_index] = FsFile();
        //log_esp3d("Closing FsFile at index %d",_index);
        _index = -1;
    }
}

ESP_SDFile  ESP_SDFile::openNextFile()
{
    if ((_index == -1) || !_isdir) {
        log_esp3d("openNextFile failed");
        return ESP_SDFile();
    }
    FsFile tmp = tSDFile_handle[_index].openNextFile();
    if (tmp) {
        char tmps[255];
        tmp.getName(tmps,254);
        log_esp3d("tmp name :%s %s", tmps, (tmp.isDir())?"isDir":"isFile");
        String s = _filename ;
        if (s!="/") {
            s+="/";
        }
        s += tmps;
        ESP_SDFile esptmp(&tmp, tmp.isDir(),false, s.c_str());
        esptmp.close();
        return esptmp;
    }
    return  ESP_SDFile();
}

const char * ESP_SD::FilesystemName()
{
    return "SDFat - " SD_FAT_VERSION_STR ;
}

#endif //SD_DEVICE == ESP_SDFAT2
#endif //ARCH_ESP32 && SD_DEVICE
