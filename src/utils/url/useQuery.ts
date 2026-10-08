import { useLocation } from "@docusaurus/router";

/**
 * HXLoLi 接入 Agent Notes v2
 * .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md
 */
export default function useQuery (): URLSearchParams {
    const location = useLocation();
    return new URLSearchParams(location.search);
}