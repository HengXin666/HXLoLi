import React, { useRef } from 'react';
import { Swiper, SwiperSlide } from 'swiper/react';
import { Autoplay, Navigation } from 'swiper/modules';
import 'swiper/css';
import 'swiper/css/navigation';
import { projects, Tags } from '@site/data/projectData';

import './css.css';
import HXLink from '../HXLink';

const ProjectCarousel: React.FC = () => {
    const swiperRef = useRef<any>(null);

    // 当项目数量较少时，复制项目数据以实现流畅的循环效果
    const duplicatedProjects = [...projects, ...projects, ...projects];

    return (
        <div
            className="project-carousel-container"
            style={{ position: 'relative', padding: '1rem 0' }}
            onMouseEnter={() => swiperRef.current?.swiper.autoplay.pause()}
            onMouseLeave={() => swiperRef.current?.swiper.autoplay.resume()}
        >
            <Swiper
                ref={swiperRef}
                modules={[Autoplay, Navigation]}
                autoplay={{
                    delay: 0, // 0延迟实现连续滚动
                    disableOnInteraction: false,
                    pauseOnMouseEnter: true // 我们手动处理暂停
                }}
                loop={true}
                slidesPerView={'auto'}  // 自动根据容器宽度计算显示数量
                spaceBetween={10}
                centeredSlides={true}   // 使滚动更流畅
                speed={2000}            // 滚动速度
                allowTouchMove={false}  // 禁止拖动触摸滑动
                draggable={false}
                style={{
                    overflow: 'visible',
                    width: '100%',
                }}
                navigation={false}      // 隐藏导航箭头
            >
                {duplicatedProjects.map((project, index) => (
                    <SwiperSlide
                        key={`${index}-${project.title}`}
                        style={{
                            width: 'auto',
                            height: 'auto',
                            // opacity: index < projects.length ? 1 : 0.6 // 原始项目更突出
                        }}
                    >
                        <div
                            className="project-card"
                            style={{
                                width: 200,
                                height: 300,
                                backgroundColor: '#22282F',
                                borderRadius: 12,
                                boxShadow: '0 4px 16px rgba(0,0,0,0.1)',
                                overflow: 'hidden',
                                transition: 'transform 0.3s ease, opacity 0.3s ease',
                            }}
                        >
                            {project.preview && (
                                <img
                                    src={project.preview}
                                    alt={project.title}
                                    style={{
                                        width: '100%',
                                        height: 140,
                                        objectFit: 'cover',
                                    }}
                                    loading="lazy"
                                />
                            )}
                            <div style={{ padding: 6 }}>
                                <HXLink
                                    title={project.title}
                                    url={project.url}
                                />
                                <p
                                    style={{
                                        fontSize: 11,
                                        WebkitLineClamp: 5, // 支持的最多行数
                                        WebkitBoxOrient: 'vertical',
                                        display: '-webkit-box',
                                        color: '#D2D7DF',
                                        overflow: 'hidden',
                                        textOverflow: 'ellipsis',
                                        textAlign: 'left',
                                        margin: 0,
                                    }}
                                >
                                    {project.description}
                                </p>
                            </div>
                            {/* 标签 */}
                            <div style={{
                                display: "flex",
                                flexWrap: "wrap",
                                gap: "8px",
                                marginTop: "2px",
                                marginLeft: "5px",
                                position: "absolute",
                                bottom: "5px",
                            }}>
                                {project.tags.map((tag, idx) => {
                                    const tagData = Tags[tag];
                                    return (
                                        <div
                                            key={idx}
                                            style={{
                                                display: "flex",
                                                alignItems: "center",
                                                padding: "2px 8px",
                                                borderRadius: "999px",
                                                backgroundColor: "rgba(255, 255, 255, 0.1)",
                                                fontSize: "9px",
                                                color: "white",
                                                cursor: "default",
                                                border: "rgba(18, 234, 223, 0.8) 1px solid"
                                            }}
                                            title={tagData.description} // @todo 以后可以优化一下弹窗
                                        >
                                            <span
                                                style={{
                                                    display: "inline-block",
                                                    width: "6px",
                                                    height: "6px",
                                                    borderRadius: "50%",
                                                    backgroundColor: tagData.color,
                                                    marginRight: "6px",
                                                }}
                                            />
                                            {tagData.label}
                                        </div>
                                    );
                                })}
                            </div>
                        </div>
                    </SwiperSlide>
                ))}
            </Swiper>

            {/* 添加渐变遮罩实现平滑过渡 */}
            <div style={{
                position: 'absolute',
                top: 0,
                left: 0,
                right: 0,
                bottom: 0,
                pointerEvents: 'none',
                background: 'linear-gradient(90deg, #2b2b2b 1%, transparent 10%, transparent 90%, #2b2b2b 99%)',
                zIndex: 10
            }} />
        </div>
    );
};

export default ProjectCarousel;