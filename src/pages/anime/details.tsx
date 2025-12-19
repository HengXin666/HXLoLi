import BlogWithCats from "@site/src/components/BlogWithCats";
import Layout from "@theme/Layout";
import React from "react";
import Heading from "@theme/Heading";
import { useLocation } from "react-router-dom";
import useBaseUrl from "@docusaurus/useBaseUrl";
import { useAnimeRecords } from "@site/src/utils/anime/animeStore";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";

function useQuery (): URLSearchParams {
    return new URLSearchParams(useLocation().search);
}

export default function AnimeDetailPage (): React.ReactNode {
    const query = useQuery();
    const id = query.get("id");

    if (!id || !/^\d+$/.test(id)) {
        return (
            <Layout title={"アニメ 404 NOT FOUND"}>
                <div style={{ height: "60px" }}></div>
                <BlogWithCats
                    style={{
                        backgroundColor: "#2b2b2b",
                        padding: "20px",
                        display: "flex",
                        flexDirection: "column",
                    }}
                >
                    <div style={{ textAlign: "center" }}>
                        <Heading as="h2" id={"404"}>
                            404 NOT FOUND
                        </Heading>
                        <img src={useBaseUrl("/anime/img/misaka-404.png")} />
                    </div>
                </BlogWithCats>
                <div style={{ height: "60px" }}></div>
            </Layout>
        );
    }

    const { siteConfig } = useDocusaurusContext();
    const records = useAnimeRecords(siteConfig.baseUrl);
    const record = records.find((r) => r.anime_data.id === parseInt(id));

    return (
        <Layout title={`${record?.anime_data.name_cn}`}>
            <div style={{ height: "60px" }}></div>
            <BlogWithCats
                style={{
                    backgroundColor: "#2b2b2b",
                    padding: "20px",
                    display: "flex",
                    flexDirection: "column",
                }}
            >
                <></>
            </BlogWithCats>
            <div style={{ height: "60px" }}></div>
        </Layout>
    );
}
