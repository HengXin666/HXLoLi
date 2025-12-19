import React from "react";
import Layout from "@theme/Layout";
import Heading from "@theme/Heading";
import { useLocation } from "react-router-dom";
import useBaseUrl from "@docusaurus/useBaseUrl";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import BlogWithCats from "@site/src/components/BlogWithCats";
import { useAnimeRecords } from "@site/src/utils/anime/animeStore";

function useQuery(): URLSearchParams {
    const location = useLocation();
    return new URLSearchParams(location.search);
}

export default function AnimeDetailPage(): React.ReactElement {
    const query = useQuery();
    const id = query.get("id");

    const { siteConfig } = useDocusaurusContext();
    const records = useAnimeRecords(siteConfig.baseUrl);

    const notFoundImageUrl = useBaseUrl("/anime/img/misaka-404.png");

    const record =
        id && /^\d+$/.test(id)
            ? records.find((r) => r.anime_data.id === Number(id))
            : undefined;

    if (!record) {
        return (
            <Layout title={"アニメ 404 NOT FOUND"}>
                <div style={{ height: "60px" }} />

                <BlogWithCats
                    style={{
                        backgroundColor: "#2b2b2b",
                        padding: "20px",
                        display: "flex",
                        flexDirection: "column",
                    }}
                >
                    <div style={{ textAlign: "center" }}>
                        <Heading as="h2" id="404">
                            404 NOT FOUND
                        </Heading>
                        <img src={notFoundImageUrl} alt="404" />
                    </div>
                </BlogWithCats>

                <div style={{ height: "60px" }} />
            </Layout>
        );
    }

    return (
        <Layout title={record.anime_data.name_cn}>
            <div style={{ height: "60px" }} />

            <BlogWithCats
                style={{
                    backgroundColor: "#2b2b2b",
                    padding: "20px",
                    display: "flex",
                    flexDirection: "column",
                }}
            >
                <h1>{record.anime_data.name_cn}</h1>
            </BlogWithCats>

            <div style={{ height: "60px" }} />
        </Layout>
    );
}
