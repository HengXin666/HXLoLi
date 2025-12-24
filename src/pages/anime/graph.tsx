import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import AnimeGraphPage from "@site/src/components/anime/AnimeGraphPage";
import Layout from "@theme/Layout";
import React from "react";

export default function AnimeGraph (): React.ReactElement {
    const { siteConfig } = useDocusaurusContext();
    return(
        <AnimeGraphPage baseUrl={siteConfig.baseUrl} />
    )
}