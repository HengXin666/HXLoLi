import React, { useEffect, useState } from 'react';
import { useLocation } from 'react-router-dom';

// @ts-ignore
import { DrawIoEmbed } from 'react-drawio';

const useQuery = () => {
    return new URLSearchParams(useLocation().search);
};

/**
 * 将 SVG 字符串转换为 Base64 格式
 * @param svgCode 
 * @returns 
 */
const convertSvgToBase64 = (svgCode: string): string => {
    // 使用TextEncoder进行Base64编码
    const encodeBase64 = (svgCode: string): string => {
        const utf8Bytes = new TextEncoder().encode(svgCode);
        return btoa(String.fromCharCode(...utf8Bytes));
    };
    return `data:image/svg+xml;base64,${encodeBase64(svgCode)}`;
};

/**
 * 将 SVG 字符串转换为 XML 格式
 * @param svgCode 
 * @returns 
 */
const convertSvgToXml = (svgCode: string): string => {
    // 解码 Base64 内容的函数
    const decodeBase64 = (base64Str: string): string => {
        const decodedData = atob(base64Str); // 解码 Base64 字符串
        return decodedData;
    };

    // 将 XML 内容转为标准的 XML 格式并添加编码声明
    const _convertToXmlFormat = (svgCode: string): string => {
        // 提取 <content> 内的 Draw.io 数据
        const contentMatch = svgCode.match(/<content>(.*?)<\/content>/s);
        if (contentMatch) {
            const encodedContent = contentMatch[1];
            
            // 检查是否为 Base64 编码的内容
            const base64Pattern = /^([A-Za-z0-9+/=]+)$/;
            if (base64Pattern.test(encodedContent)) {
                const decodedContent = decodeBase64(encodedContent);
                
                // 创建 XML 声明并返回
                const xmlDeclaration = `<?xml version="1.0" encoding="UTF-8"?>\n`;
                return xmlDeclaration + decodedContent;
            }
        }

        // 如果没有找到 Base64 编码内容, 直接返回 SVG 格式
        return svgCode;
    };

    return _convertToXmlFormat(svgCode);
};

const DrawIoPage: React.FC = () => {
    const query = useQuery();
    const src = query.get('src'); // 获取 ?src 参数
    const [initialXML, setInitialXML] = useState<string | null>(null);

    // 当 src 存在时, 加载并转换为 Base64 格式
    useEffect(() => {
        const fetchSvg = async (svgUrl: string) => {
            try {
                // 获取 SVG 文件
                const response = await fetch(svgUrl);
                const svgText = await response.text(); // 获取 SVG 文件的文本内容
                setInitialXML(convertSvgToXml(svgText));
            } catch (error) {
                console.error('加载 SVG 文件失败:', error);
            }
        };

        if (src) {
            fetchSvg(src);
        }
    }, [src]);

    return (
        <div style={{ height: '100vh', display: 'flex', flexDirection: 'column' }}>
            {initialXML ? (
                    <DrawIoEmbed
                        xml={initialXML} // 传递 Base64 编码的 SVG 数据
                        urlParameters={{
                            ui: 'dark',
                            spin: true,
                            libraries: true,
                            saveAndExit: false,
                            noSaveBtn: true, // 保存ボタンを非表示にする
                            noExitBtn: true, // 終了ボタンを非表示にする
                        }}
                        
                    />
            ) : (
                <p>未找到图形文件: {src}</p>
            )}
        </div>
    );
};

export default DrawIoPage;
