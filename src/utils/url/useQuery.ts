import { useLocation } from "@docusaurus/router";

export default function useQuery (): URLSearchParams {
    const location = useLocation();
    return new URLSearchParams(location.search);
}