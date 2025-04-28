# TagLib

> [!TIP]
> 大多数 Linux 发行版都预装了 TagLib

[[Github] taglib](https://github.com/taglib/taglib)

```sh
sudo pacman -S taglib
```

---

```C++
#include <taglib/fileref.h>
#include <taglib/tag.h>
#include <taglib/audioproperties.h>

#include <taglib/mpegfile.h>

#include <taglib/id3v2tag.h>
#include <taglib/attachedpictureframe.h>

#include <taglib/flacfile.h>
#include <taglib/flacpicture.h>

#include <taglib/mp4file.h>
#include <taglib/mp4tag.h>
#include <taglib/mp4item.h>
#include <taglib/mp4coverart.h>

#include <taglib/asffile.h>
#include <taglib/asftag.h>

#include <QFileInfo>
#include <QPixmap>

#include <utils/FileInfo.hpp>

namespace HX {

namespace internal {

/**
 * @brief 支持的文件类型
 */
inline static const QSet<QString> extensionSet {
    "mp3",
    "wav",
    "flac",
    "ogg",
    "mpc",
    "spx",
    "wv",
    "tta",
    "aiff",
    "aif",
    "mp4",
    "ape",
    "asf",
    "dsf",
    "dff",
    "acc",
};

} // namespace internal

class MusicInfo {
public:
    explicit MusicInfo(QFileInfo const& fileInfo)
        : _fileInfo(fileInfo)
        , _byteArr(QFile::encodeName(_fileInfo.canonicalFilePath()))
        , mpegFile(_byteArr.constData())
    {}

    /**
     * @brief 是否不支持该格式
     * @param fileInfo 
     * @return true 是不支持
     * @return false 不是不支持
     */
    inline static bool isNotSupport(QFileInfo const& fileInfo) {
        return internal::extensionSet.find(fileInfo.suffix().toLower()) 
            == internal::extensionSet.end();
    }

    /**
     * @brief 获取音频标题
     * @return QString `获取失败`则返回文件名
     */
    QString getTitle() const {
        if (mpegFile.isNull() || !mpegFile.tag()) {
            return _fileInfo.fileName();
        }
        auto res = QString::fromStdString(mpegFile.tag()->title().to8Bit(true));
        return res.isEmpty() 
            ? _fileInfo.fileName() 
            : res;
    }

    /**
     * @brief 获取音频歌手信息
     * @param errRes 获取失败返回的值
     * @return QString 
     */
    QString getArtist(QString&& errRes = "") const {
        if (mpegFile.isNull() || !mpegFile.tag()) {
            return errRes;
        }
        return QString::fromStdString(mpegFile.tag()->artist().to8Bit(true));
    }

    /**
     * @brief 获取音频专辑信息
     * @param errRes 获取失败返回的值
     * @return QString 
     */
    QString getAlbum(QString&& errRes = "") const {
        if (mpegFile.isNull() || !mpegFile.tag()) {
            return errRes;
        }
        return QString::fromStdString(mpegFile.tag()->album().to8Bit(true));
    }

    /**
     * @brief 格式化音频时长为`HH:MM:SS`格式, 如`3:14`(不含前导0)
     * @param errRes 获取失败返回的值
     * @return QString 
     */
    QString formatTimeLengthToHHMMSS(QString&& errRes = "") const {
        if (mpegFile.isNull() || !mpegFile.audioProperties()) {
            return errRes;
        }
        TagLib::AudioProperties* properties = mpegFile.audioProperties();
        int time = properties->lengthInSeconds();
        int hTime = time / 3600;
        return QString{"%1%2:%3"}
            .arg(hTime 
                    ? QString{"%1:"}.arg(hTime) 
                    : "")
            .arg(time / 60 % 60)
            .arg(time % 60);
    }

    // 高性能获取专辑图片函数，支持 MP3、FLAC、MP4/M4A 格式
    QPixmap getAlbumArtAdvanced() {
        QPixmap pixmap;
        QString ext = _fileInfo.suffix().toLower();

        // 1. MP3（及部分 AAC 封装在 MP3 容器中）处理: 使用 ID3v2
        // 处理 MP3 文件的专辑图片
        if (ext == "mp3" || ext == "aac") {
            TagLib::MPEG::File mpegFile{_byteArr.constData()};
            TagLib::ID3v2::Tag *id3v2Tag = mpegFile.ID3v2Tag(true);
            if (id3v2Tag) {
                // 先查找APIC帧
                TagLib::ID3v2::FrameList frames = id3v2Tag->frameList("APIC");
                // 如果APIC帧为空，再尝试PIC帧（ID3v2.2的情况）
                if (frames.isEmpty())
                    frames = id3v2Tag->frameList("PIC");
                if (!frames.isEmpty()) {
                    // 取第一个帧的数据
                    TagLib::ID3v2::AttachedPictureFrame *picFrame =
                        dynamic_cast<TagLib::ID3v2::AttachedPictureFrame*>(frames.front());
                    if (picFrame) {
                        const TagLib::ByteVector& pictureData = picFrame->picture();
                        QByteArray ba(pictureData.data(), static_cast<int>(pictureData.size()));
                        QPixmap pixmap;
                        pixmap.loadFromData(ba);
                        return pixmap;
                    }
                }
            }
        }
        // 2. FLAC处理
        else if (ext == "flac") {
            TagLib::FLAC::File flacFile(_byteArr.constData());
            if (flacFile.isValid() && !flacFile.pictureList().isEmpty()) {
                const TagLib::FLAC::Picture *pic = flacFile.pictureList().front();
                if (pic) {
                    const TagLib::ByteVector& bv = pic->data();
                    QByteArray ba(bv.data(), static_cast<int>(bv.size()));
                    pixmap.loadFromData(ba);
                    return pixmap;
                }
            }
        }
        // 3. MP4/M4A处理
        else if (ext == "mp4" || ext == "m4a") {
            TagLib::MP4::File mp4File(_byteArr.constData());
            if (mp4File.isValid() && mp4File.tag()) {
                TagLib::MP4::Tag *mp4Tag = mp4File.tag();
                TagLib::MP4::ItemMap items = mp4Tag->itemMap();
                if (items.contains("covr")) {
                    TagLib::MP4::CoverArtList coverList = items["covr"].toCoverArtList();
                    if (!coverList.isEmpty()) {
                        TagLib::MP4::CoverArt cover = coverList.front();
                        const TagLib::ByteVector& bv = cover.data();
                        QByteArray ba(bv.data(), static_cast<int>(bv.size()));
                        pixmap.loadFromData(ba);
                        return pixmap;
                    }
                }
            }
        }
        // 其他格式暂未实现或没有嵌入专辑图片，返回空QPixmap
        return pixmap;
    }

private:
    QFileInfo const& _fileInfo;
    QByteArray _byteArr;
    TagLib::FileRef mpegFile;
};

} // namespace HX
```
