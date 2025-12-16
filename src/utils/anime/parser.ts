import { WatchStatus, SubjectType, ANiMeRecord, Actor } from './types';

// 定义一个类型, 用于描述从 JSON.parse 出来的、可能带有 __dataclass__ 的原始对象
type RawObject = { [key: string]: any };

/**
 * 递归地解析一个对象, 将其中的 __dataclass__ 结构和 enum 值进行转换。
 * 这等价于 Python 中的 ANiMeDecoder.object_hook 的功能。
 * @param obj 从 JSON.parse 出来的原始对象或值
 * @returns "复活"后的对象
 */
function revive(obj: any): any {
  // 如果不是对象, 或者是 null, 则直接返回 (递归的基线条件)
  if (typeof obj !== 'object' || obj === null) {
    return obj;
  }

  // 递归地处理数组中的每个元素
  if (Array.isArray(obj)) {
    return obj.map(item => revive(item));
  }

  // 递归地处理对象的所有属性值
  // 确保我们从最内层的对象开始转换
  const revivedObj: RawObject = {};
  for (const key in obj) {
    if (Object.prototype.hasOwnProperty.call(obj, key)) {
      revivedObj[key] = revive(obj[key]);
    }
  }

  // 在所有子属性都被"复活"后, 处理当前对象
  if ('__dataclass__' in revivedObj) {
    const { __dataclass__, ...data } = revivedObj;

    // 在这里, 你可以根据 __dataclass__ 的值进行特定的类型转换,
    // 但在TS中, 主要工作是处理enum。
    // 因为TS的类型系统在编译时起作用, 运行时我们无法像Python那样动态创建类实例。
    // 不过, 我们可以确保enum字段的值是正确的enum成员。

    switch (__dataclass__) {
      case 'Relation':
        if ('type' in data && typeof data.type === 'number') {
          // 将数字转换为 SubjectType enum 成员
          data.type = data.type as SubjectType;
        }
        break;

      case 'UserStatus':
        if ('watch_status' in data && typeof data.watch_status === 'number') {
          // 将数字转换为 WatchStatus enum 成员
          data.watch_status = data.watch_status as WatchStatus;
        }
        break;

      // ANiMeRecord, ANiMeData, Character, Actor 等类, 其字段在递归中已处理完毕
      // 不需要额外操作, 直接返回数据部分即可
    }

    // 返回处理过 enum 后的数据对象
    return data;
  }

  // 如果没有 __dataclass__ 字段, 返回已经递归处理过子属性的对象
  return revivedObj;
}


/**
 * 解析包含 __dataclass__ 信息的 JSON 字符串, 返回类型安全的对象。
 * @param jsonString 待解析的 JSON 字符串
 * @returns 经过类型和 enum 值转换后的对象
 */
export function parse<T>(jsonString: string): T {
  // 使用 JSON.parse 的第二个参数(reviver)效率更高, 但为了模仿 Python 的 object_hook
  // 递归结构, 我们采用后处理的方式, 这样逻辑更清晰, 更易于理解和调试。
  const rawData = JSON.parse(jsonString);
  return revive(rawData) as T;
}
