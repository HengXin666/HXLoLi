/**
 * 匹配一个正则表达式
 * @param regexStr 正则表达式, 要求仅有一个匹配项目
 * @param input 需要匹配的字符串
 * @returns 匹配的内容或者没有匹配到, 返回 undefined
 */
function matchRegex(regexStr: string, input: string | undefined): string | undefined {
    if (!input)
        return undefined;
    const match = input.match(new RegExp(regexStr));
    return match ? match[1] : undefined;
}

export default matchRegex;
