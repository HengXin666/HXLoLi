import React, { useEffect, useState } from "react";
import { ReactNode } from "react";
import Layout from "@theme/Layout";

import AnimeCard from "@site/src/components/anime/AnimeCard";
import { parse } from "@site/src/utils/anime/parser"
import { ANiMeRecord } from "@site/src/utils/anime/types";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";

export default function Home(): React.ReactNode {
  const { siteConfig } = useDocusaurusContext();

  // 1. 为存储数据和加载状态创建 state
  const [animeRecords, setAnimeRecords] = useState<ANiMeRecord[]>([]);
  const [isLoading, setIsLoading] = useState(true);

  // 2. 使用 useEffect 来执行只运行一次的副作用（数据获取）
  useEffect(() => {
    async function fetchRecords() {
      try {
        // 构建文件的公共 URL
        const url = `${siteConfig.baseUrl}anime/ANiMeRecord.json`; // 假设文件在 static/anime/ 目录下
        
        // 发起异步网络请求
        const response = await fetch(url);
        if (!response.ok) {
          throw new Error(`Failed to fetch: ${response.statusText}`);
        }
        
        // 获取 JSON 文件的文本内容
        const jsonString = await response.text();
        
        // 使用你自己的解析器来处理 __dataclass__ 和 enum
        const data = parse<ANiMeRecord[]>(jsonString);
        
        // 3. 将获取并解析好的数据存入 state
        setAnimeRecords(data);

      } catch (error) {
        console.error("Error fetching anime records:", error);
      } finally {
        // 无论成功还是失败，都结束加载状态
        setIsLoading(false);
      }
    }

    // 调用异步函数
    fetchRecords();
  }, []); // 空依赖数组 `[]` 保证这个 effect 只在组件首次挂载时运行一次

  // 4. 根据加载状态和数据来渲染 UI
  return (
    <Layout title={"アニメ"}>
      <main>
        <div className="container" style={{ marginTop: '2rem' }}>
          {isLoading ? (
            <p>正在加载...</p>
          ) : (
            animeRecords.map((record: ANiMeRecord) => (
              <AnimeCard key={record.anime_data.id} record={record} />
            ))
          )}
        </div>
      </main>
    </Layout>
  );
}